#!/usr/bin/python
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
from fast_mobile_localization.msg import Sensor
from robot_controller.msg import Gps_msg
from visualization_msgs.msg import Marker

from geometry_msgs.msg import Pose2D



from nav_msgs.msg import MapMetaData
from models.rssi_model import RSSIModel
from localization_ekf import LocalizationEKF
from localization_ukf import LocalizationUKF
import time
import yaml
import pickle
import utm
from models.real_sensors import DoubleDecawaveReal, DoubleDecawaveGroundTruth

import signal
import sys
   

class Robot:
    def __init__(self):

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
        self.odom_pub2 = rospy.Publisher('odom_world', Odometry, queue_size=10)


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
        self.odom_pub2.publish(odom)




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
            self.getGroundTruthPose()
            time.sleep(0.5)

        return self.initial_pose



# def signal_handler(sig, frame):

#         sys.exit(0)


if __name__ == "__main__":
    # signal.signal(signal.SIGINT, signal_handler)
    # signal.pause()


    rospy.init_node('fast_mobile_localization_real', anonymous=True)


    devices = [('/dwc0af/tag_status', -0.225), ('/dwc4b8/tag_status', 0.225)]

    covariance_matrix = np.matrix([[0.135148,  0.0559928], \
                                    [0.0559928, 0.1682208]])


    odom_alphas = [0.001, 0.05, 0.001, 0.05]
    robot = Robot()

    # #create the localization module
    initial_pose = robot.getInitialPose()



    devices_gt = [('/dwc0af/tag_pose', -0.225), ('/dwc4b8/tag_pose', 0.225)]
    decawave_ground_truth = DoubleDecawaveGroundTruth(devices_gt)

    GPS_           = None 
    Decawave_      = None
    DoubleSensors_ = None
    Sensors_        = None 
    DoubleDecawave_ = None 


    algorithms = ['EKF', 'UKF']

    sensor_list = [['DoubleDecawaveReal']]


    localization_algs = []
    for algorithm in algorithms:           
        for sensor_set in sensor_list:

            if(algorithm == 'UKF'):
                localization = LocalizationUKF(initial_pose, odom_alphas)

            elif(algorithm == 'EKF'):
                localization = LocalizationEKF(initial_pose, odom_alphas)

            else:
                
                print('ERROR - Algorithm --' + algorithm+ '-- not implemented')
                exit()

            sensor_str = algorithm
            sensor_obj = []
            for sensor in sensor_set:
    
                if(sensor == 'DoubleDecawaveReal'):
                    
                    DoubleDecawave_ = DoubleDecawaveReal(devices, covariance_matrix)
                    sensor_ = DoubleDecawave_

                else:
                    print('ERROR - sensor --' + sensor + '-- not implemented')
                    exit()

                sensor_obj.append(sensor_)
                sensor_str += "_" + sensor
                sensor_.addCallback(localization.receiveSensorData)

            robot.addCallback(localization.receiveOdom)
            localization_algs.append((localization, sensor_str, sensor_obj))


    rate = rospy.Rate(25.0)
    data = {}
    data['ground_truth'] = {}
    data['ground_truth']['x'] = []
    data['ground_truth']['y'] = []
    data['ground_truth']['t'] = []

    #
    data['odom'] = {}
    data['odom']['x'] = []
    data['odom']['y'] = []
    data['odom']['t'] = []


    data['deca_gt'] = {}
    data['deca_gt']['x'] = []
    data['deca_gt']['y'] = []
    data['deca_gt']['t'] = []

    for localization_alg in localization_algs:
        data[localization_alg[1]] = {}
        data[localization_alg[1]]['x'] = []
        data[localization_alg[1]]['y'] = []
        data[localization_alg[1]]['t'] = []


    rate = rospy.Rate(25.0)

    while not rospy.is_shutdown():
        

        #ground truth
        pose = robot.getGroundTruthPose()
        data['ground_truth']['x'].append(pose[0])
        data['ground_truth']['y'].append(pose[1])
        data['ground_truth']['t'].append(pose[2])
        posegt = pose

        #
        pose = robot.getCorrectedOdom()
        data['odom']['x'].append(pose[0])
        data['odom']['y'].append(pose[1])
        data['odom']['t'].append(pose[2])



        pose = decawave_ground_truth.getPose()
        data['deca_gt']['x'].append(pose[0])
        data['deca_gt']['y'].append(pose[1])
        data['deca_gt']['t'].append(pose[2])

        #print(data['deca_gt']['t'][-1], data['ground_truth']['t'][-1])


        for localization_alg in localization_algs:

            pose = localization_alg[0].getPose()

            data[localization_alg[1]]['x'].append(pose[0])
            data[localization_alg[1]]['y'].append(pose[1])
            data[localization_alg[1]]['t'].append(pose[2])


        #print(posegt, pose)

        rate.sleep()

    name = '/home/elerson/.ros/test_real_'+str(time.time())[:-3]
    with open(name + '.pkl', 'wb') as f:
        pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)
        

