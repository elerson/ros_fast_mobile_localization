#!/usr/bin/python
import rospy
from scipy.optimize import linear_sum_assignment

import tf
import numpy as np
np.random.seed(0)

import math
from math import cos, sin, atan2, sqrt
import threading


#ROS Imports
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion 
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData
from models.rssi_model import RSSIModel
from localization import Localization
from localization_ukf import  LocalizationUKF
import time
import yaml
from numpy.linalg import cholesky
from models.sensors import Sensors, DoubleSensors, GPS, DoubleDecawave, Decawave
import os




from enum import IntEnum
class COLLISION(IntEnum): 
    NONE           = 0   
    VERTICAL       = 1
    HORIZONTAL     = 2


class Robot:
    def __init__(self, odom_alphas):
        #subscribers
        rospy.Subscriber("odom", Odometry, self.odomCallback)
        rospy.Subscriber("base_pose_ground_truth", Odometry, self.groundTruthCallback)

        #publishers
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/map_metadata", MapMetaData, self.getMapMetadata)
        self.odom_pub = rospy.Publisher('simulated_odom', Odometry, queue_size=10)

        self.start_map = False
        self.start_grounf_truth = False

        self.goal_direction = np.random.random(1)[0]*2*math.pi
        self.look_ahead = 3.0

        self.current_odom_time = time.time()
        self.current_odom = np.matrix([[0],[0],[0]])
        self.current_pose = np.matrix([[0],[0],[0]])

        self.first_odom = True

        self.alpha1 = odom_alphas[0]
        self.alpha2 = odom_alphas[1]
        self.alpha3 = odom_alphas[2]
        self.alpha4 = odom_alphas[3]

        self.callback = None

        #wait until the map parameters and the ground truth are received
        while(not (self.start_map and self.start_grounf_truth)):
            time.sleep(0.1)


    def getMapMetadata(self, map_data):
        self.map_resolution = map_data.resolution
        self.height         = map_data.height
        self.width         = map_data.width

        self.map_max_x = self.width*self.map_resolution
        self.map_min_x = 0

        self.map_max_y = self.height*self.map_resolution
        self.map_min_y = 0

        self.start_map = True




    def verifyCollision(self):
        x_ = self.position_ground_truth.x
        y_ = self.position_ground_truth.y

        current_direction = self.goal_direction + 0.2#self.orientation_ground_truth[2] +0.2

        x = x_ + cos(current_direction)*self.look_ahead
        y = y_ + sin(current_direction)*self.look_ahead

        if(x > self.map_max_x or x < self.map_min_x):
            return COLLISION.VERTICAL

        if(y > self.map_max_y or y < self.map_min_y):
            return COLLISION.HORIZONTAL

        current_direction = self.goal_direction - 0.2


        x = x_ + cos(current_direction)*self.look_ahead
        y = y_ + sin(current_direction)*self.look_ahead

        if(x > self.map_max_x or x < self.map_min_x):
            return COLLISION.VERTICAL

        if(y > self.map_max_y or y < self.map_min_y):
            return COLLISION.HORIZONTAL


        return COLLISION.NONE



    def select_direction(self, collision_type):

        current_direction = self.goal_direction#self.orientation_ground_truth[2]
        new_direction = 0
        if(collision_type == COLLISION.VERTICAL):
            if current_direction > math.pi/2 and current_direction < 3*math.pi/2:
                new_direction = (np.random.random(1)[0]*np.pi + 3*np.pi/2)%2*math.pi
            else:
                new_direction = (np.random.random(1)[0]*np.pi + np.pi/2)%2*math.pi
        elif(collision_type == COLLISION.HORIZONTAL):
            if current_direction > 0 and current_direction < math.pi:
                new_direction = (np.random.random(1)[0]*np.pi + np.pi)%2*math.pi
            else:
                new_direction = (np.random.random(1)[0]*np.pi)%2*math.pi

        return new_direction


    def randomWalk(self):
        #test colision
        collision = self.verifyCollision()
        if(collision != COLLISION.NONE):
            self.goal_direction = self.select_direction(collision)

        #        
        m = 0.9
        final_direction = (m*cos(self.goal_direction), m*sin(self.goal_direction))
        robot_angle = self.orientation_ground_truth[2]
    
        theta = 0.8*(final_direction[1]*math.cos(robot_angle) - final_direction[0]*math.sin(robot_angle))
        linear = final_direction[0]*math.cos(robot_angle) + final_direction[1]*math.sin(robot_angle)
    
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.linear.y = 0        
        cmd_vel.angular.z = theta
        
        self.vel_pub.publish(cmd_vel)



    def odomCallback(self, odom):
        self.position = odom.pose.pose.position

        orientation = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)

        self.orientation = tf.transformations.euler_from_quaternion(orientation)
        pass

    def groundTruthCallback(self, odom):

        self.position_ground_truth = odom.pose.pose.position

        orientation = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)

        self.orientation_ground_truth = tf.transformations.euler_from_quaternion(orientation)

        if(not self.start_grounf_truth):
            self.current_pose = np.matrix([[self.position_ground_truth.x],[self.position_ground_truth.y],[self.orientation_ground_truth[2]]])


        self.start_grounf_truth = True
        self.defineOdom()

        pass


    def getVelocities(self, last_odom, new_odom, last_time, new_time):

        x = new_odom[0,0] - last_odom[0,0]
        y = new_odom[1,0] - last_odom[1,0]

        direction =  cos(last_odom[2,0])*x + sin(last_odom[2,0])*y +0.000000000001
        direction /= abs(direction)

        delta_time = new_time - last_time

        v = direction*math.sqrt(x**2 + y**2)/delta_time
        w = (new_odom[2,0] - last_odom[2,0])/delta_time
        w = atan2(sin(w), cos(w))


        return (v, w), delta_time


    def defineOdom(self):

        odom = self.getGroundTruthPose()
        new_odom = np.matrix([[odom[0]],[odom[1]],[odom[2]]])
        new_odom_time = time.time()
        if( not self.first_odom):
            #defining the velocities for the odometry
            velocities, delta_time = self.getVelocities(self.current_odom, new_odom, self.current_odom_time, new_odom_time)

            if( not (velocities[0]==0.0 and velocities[1] == 0.0)):
                #print(velocities, delta_time)
                v_t = velocities[0] + np.random.normal(0, (velocities[0]**2)*self.alpha1 + (velocities[1]**2)*self.alpha2) 
                w_t = velocities[1] + np.random.normal(0, (velocities[0]**2)*self.alpha3 + (velocities[1]**2)*self.alpha4)

            else:
                v_t = velocities[0]
                w_t = velocities[1]

            theta = self.current_pose[2, 0]
            if(w_t != 0.0):
                #using the velocities, calculating the new pose
                self.current_pose = self.current_pose + np.matrix([[-(v_t/w_t)*sin(theta) + (v_t/w_t)*sin(theta + w_t*delta_time)], \
                                                   [(v_t/w_t)*cos(theta)  - (v_t/w_t)*cos(theta + w_t*delta_time)], \
                                                   [w_t*delta_time]])

        self.first_odom = False
        
        self.current_odom = new_odom
        self.current_odom_time = new_odom_time

        if(self.callback != None):
            self.callback((self.current_pose[0, 0], self.current_pose[1, 0], self.current_pose[2, 0]))

        self.publishOdom()
        

    def getOdom(self):
        return (self.current_pose[0, 0], self.current_pose[1, 0], self.current_pose[2, 0])

    def addCallback(self, callback):
        self.callback = callback

    def publishOdom(self):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/odom' # i.e. '/odom'
        

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

        # Publish odometry message
        self.odom_pub.publish(msg)

    def getGroundTruthPose(self):
        return (self.position_ground_truth.x, self.position_ground_truth.y, self.orientation_ground_truth[2])


    def run(self):
        self.randomWalk()
        pass


if __name__ == "__main__":

    rospy.init_node('fast_mobile_localization', anonymous=True)
    config_file = rospy.get_param("~config_file")

    odom_alphas = [0.2, 0.2, 0.2, 0.2]
    robot   = Robot(odom_alphas)
    #sensors = Sensors(robot, config_file)
    sensors = DoubleDecawave(robot, config_file) #DoubleDecawave(robot, config_file)#DoubleSensors(robot, config_file)
    gps     = GPS(robot, 0.01)
 
    #create the localization module
    initial_pose = robot.getGroundTruthPose()
    localization = LocalizationUKF(initial_pose, odom_alphas, 3)

    #add the data callbacks to the localization modules
    #double_sensors.addCallback(localization.receiveDoubleSensorData)
    sensors.addCallback(localization.receiveSensorData)
    robot.addCallback(localization.receiveOdom)
    gps.addCallback(localization.receiveSensorData)

    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():
        robot.run()
        #sensors.run()
        rate.sleep()