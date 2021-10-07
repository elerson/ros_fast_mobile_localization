#!/usr/bin/python3
import rospy
from scipy.optimize import linear_sum_assignment

import tf
import numpy as np
import math
from math import cos, sin, log10, sqrt, atan2
import threading


#ROS Imports
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion 
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker


from geometry_msgs.msg import Pose2D
from ros_decawave.msg import Tag


from nav_msgs.msg import MapMetaData
from models.rssi_model import RSSIModel
from localization_ekf import LocalizationEKF
from localization_ukf import LocalizationUKF
#from localization_rsf import LocalizationRSF

import time
import yaml
import pickle
from models.real_sensors import DecawaveReal, DecawaveRSF

import signal
import sys
   

class Robot:
    def __init__(self, tag_sub):

        self.first_odom = True
        self.odom_callback = []
        self.odom = (0, 0, 0)
        self.ground_truth = (0, 0, 0)
        self.tf_listener = tf.TransformListener()

        self.initial = False
        self.initial_pose = (0,0,0)

        self.current_odom_time = time.time()
        self.current_odom = np.matrix([[0],[0],[0]])
        self.current_pose = np.matrix([[0],[0],[0]])

        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        self.odom_pub = rospy.Publisher('simulated_odom', Odometry, queue_size=10)        
        rospy.Subscriber(tag_sub, Tag, self.decawavePos)

    def decawavePos(self, data):
    	self.initial = True
    	self.initial_pose = (data.x, data.y, 0, 0) 

    def publishOdom(self):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/world' # i.e. '/odom'
        

        msg.pose.pose.position = Point(self.current_pose[0, 0], self.current_pose[1, 0], 0.0)
        #print(msg.pose.pose.position)

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.current_pose[2,0])

        #type(pose) = geometry_msgs.msg.Pose
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        p_cov = np.array([0.0]*36).reshape(6,6)
        msg.pose.covariance = tuple(p_cov.ravel().tolist())

        for callback in self.odom_callback:
            callback((self.current_pose[0, 0], self.current_pose[1, 0] , self.current_pose[2, 0]))

        

        # Publish odometry message
        self.odom_pub.publish(msg)

        
    def getVelocities(self, last_odom, new_odom, last_time, new_time):

        x = new_odom[0,0] - last_odom[0,0]
        y = new_odom[1,0] - last_odom[1,0]

        direction =  cos(last_odom[2,0])*x + sin(last_odom[2,0])*y +0.000000001
        direction /= abs(direction)

        delta_time = new_time - last_time

        v = direction*math.sqrt(x**2 + y**2)/delta_time
        w = (new_odom[2,0] - last_odom[2,0])/delta_time
        w = atan2(sin(w), cos(w)) + 0.0000001

        return (v, w), delta_time


    def defineOdom(self):

        if(not self.initial):
            return

        odom = self.getOdom()
        new_odom = np.matrix([[odom[0]],[odom[1]],[odom[2]]])
        new_odom_time = time.time()
        if( not self.first_odom and (new_odom_time - self.current_odom_time) > 0.2):
            #defining the velocities for the odometry
            #print(self.current_odom, new_odom)
            velocities, delta_time = self.getVelocities(self.current_odom, new_odom, self.current_odom_time, new_odom_time)

            v_t = velocities[0]
            w_t = velocities[1]
            #print(velocities, delta_time)
            #print(delta_time)
            theta = self.current_pose[2, 0]
            
                #using the velocities, calculating the new pose
            self.current_pose = self.current_pose + np.matrix([[-(v_t/w_t)*sin(theta) + (v_t/w_t)*sin(theta + w_t*delta_time)], \
                                                   [(v_t/w_t)*cos(theta)  - (v_t/w_t)*cos(theta + w_t*delta_time)], \
                                                   [w_t*delta_time]])


            self.current_odom = new_odom
            self.current_odom_time = new_odom_time
        elif((new_odom_time - self.current_odom_time) > 0.2):
            self.current_pose = np.matrix([[self.initial_pose[0]],[self.initial_pose[1]],[self.initial_pose[2]]])
            self.current_odom = new_odom
            self.current_odom_time = new_odom_time



        self.first_odom = False
        self.publishOdom()
        

    def groundTruthCallback(self, pose):
        self.ground_truth = (pose.x, pose.y, pose.theta) 


    def odomCallback(self, odom):
        self.position = odom.pose.pose.position

        orientation = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)

        self.orientation = tf.transformations.euler_from_quaternion(orientation)


        self.odom = (self.position.x, self.position.y , self.orientation[2])

        self.defineOdom()

        odom.header.frame_id = '/world'




    def addCallback(self, callback):
        self.odom_callback.append(callback)

    def getCorrectedOdom(self):
        return (self.current_pose[0,0], self.current_pose[1,0], self.current_pose[2,0]) 

    def getOdom(self):
        return self.odom

    def getGroundTruthPose(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform("/world", "/base_link", rospy.Time(0))
            orientation = tf.transformations.euler_from_quaternion(rot)
            #print('initial')
            if(not self.initial):
                self.initial_pose = (trans[0], trans[1], orientation[2])
                self.initial = True

        except:
            (trans,orientation) = (0,0,0), (0,0,0)
       
        return (trans[0],trans[1],orientation[2])

    def getInitialPose(self):

        while(not self.initial):
            
            time.sleep(0.5)

        return self.initial_pose




def publish_odom(tf_pub, publisher, pose):
  msg = Odometry()
  msg.header.stamp = rospy.Time.now()

        

  msg.pose.pose.position = Point(pose[0], pose[1],  0)
  #print(msg.pose.pose.position)

  quaternion = tf.transformations.quaternion_from_euler(0, 0, pose[2])

  #type(pose) = geometry_msgs.msg.Pose
  msg.pose.pose.orientation.x = quaternion[0]
  msg.pose.pose.orientation.y = quaternion[1]
  msg.pose.pose.orientation.z = quaternion[2]
  msg.pose.pose.orientation.w = quaternion[3]

  p_cov = np.array([0.0]*36).reshape(6,6)
  msg.pose.covariance = tuple(p_cov.ravel().tolist())

  # Publish odometry message
  
  
  base_frame_id = "base_link_uwb"
  odom_frame_id = "map"
    
  msg.header.frame_id = odom_frame_id
  publisher.publish(msg)
  quat = tf.transformations.quaternion_from_euler(0, 0, 0)
  tf_pub.sendTransform((0, 0, 0), quat, msg.header.stamp, base_frame_id, odom_frame_id)


def getInitialPose(data):
    global initial_pose
    yaw = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                                         data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

    initial_pose = (data.pose.pose.position.x, data.pose.pose.position.y, yaw)
    pass

# def signal_handler(sig, frame):

#         sys.exit(0)


if __name__ == "__main__":
    # signal.signal(signal.SIGINT, signal_handler)
    # signal.pause()
    
    rospy.init_node('fast_mobile_localization_real', anonymous=True)
    odom_pub = rospy.Publisher('uwb_odom', Odometry, queue_size=10)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, getInitialPose)
    global initial_pose


    tf_pub = tf.TransformBroadcaster()
    
    devices = ('/tag_status', 0.15)

    covariance_matrix = np.matrix([[0.1]])

    conf  = 0.01
    odom_alphas = [0.001*conf, 0.05*conf, 0.001*conf, 0.05*conf]
    robot = Robot('/tag_pose')

    # #create the localization module
    #initial_pose = robot.getInitialPose()
    initial_pose = None   
    wheel_baseline = 0.06
    
    DecawaveReal = DecawaveReal(devices, covariance_matrix)
    #DecawaveReal = DecawaveRSF(devices, covariance_matrix[0, 0])
    
    
    rate = rospy.Rate(25.0)

    initialized = False
    while not rospy.is_shutdown():
    
        if (initial_pose != None):
            localization = LocalizationEKF(initial_pose, odom_alphas)
            #localization = LocalizationRSF(initial_pose, wheel_baseline, (0.0015, 0.0015, 0.001))

            DecawaveReal.addCallback(localization.receiveSensorData)
            robot.addCallback(localization.receiveOdom)

            print('initial_pose', initial_pose)
            initial_pose = None
            initialized = True


        if initialized:
            pose = localization.getPose()
            publish_odom(tf_pub, odom_pub, pose)

        #print(pose)
        rate.sleep()

