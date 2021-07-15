#!/usr/bin/python

import numpy as np
#np.random.seed(0)

import rospy
from scipy.optimize import linear_sum_assignment

import tf

import math
from math import cos, sin, atan2
import threading
import sys
import subprocess as sub


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
import time
import yaml
import pickle
from models.sensors import Sensors, DoubleSensors, GPS

from enum import IntEnum

class COLLISION(IntEnum): 
    NONE           = 0   
    VERTICAL       = 1
    HORIZONTAL     = 2



class Robot:
    def __init__(self, odom_alphas):
        self.first_odom = True
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

        

        self.alpha1 = odom_alphas[0]
        self.alpha2 = odom_alphas[1]
        self.alpha3 = odom_alphas[2]
        self.alpha4 = odom_alphas[3]

        self.callback = []
        self.ended = False

        #wait until the map parameters and the ground truth are received
        while(not (self.start_map and self.start_grounf_truth)):
            time.sleep(0.1)
        t = 21
        self.change_direction_time = [t,2*t]
        self.time = time.time()
    
    def resetTime(self):
        self.time = time.time()

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



    def select_direction(self, ):
        

        t = time.time() - self.time 
        for i in range(2):
            if t < self.change_direction_time[i]:
                if i%2 == 0:
                    return self.orientation_ground_truth[2] + math.pi/8
                else:
                    return self.orientation_ground_truth[2] - math.pi/8

        self.ended = True
        return 0

    def isEnded(self):
        return self.ended

    def randomWalk(self):
        #test colision

        self.goal_direction = self.select_direction()
        print(self.goal_direction)

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

        direction =  cos(last_odom[2,0])*x + sin(last_odom[2,0])*y +0.000000001
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

        for callback in self.callback:
            callback((self.current_pose[0, 0], self.current_pose[1, 0], self.current_pose[2, 0]))
        #print( np.random.normal(0,1))
        self.publishOdom()
        

    def getOdom(self):
        return (self.current_pose[0, 0], self.current_pose[1, 0], self.current_pose[2, 0])

    def addCallback(self, callback):
        self.callback.append(callback)

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

    odom_alphas = [0.1, 0.1, 0.2, 0.2]
    robot = Robot(odom_alphas)
    sensors = Sensors(robot, config_file)

    #create the localization module
    initial_pose  = robot.getGroundTruthPose()
    localization_sensors  = Localization(initial_pose, odom_alphas)
    localization_gps  = Localization(initial_pose, odom_alphas)
    localization_gps_sensor  = Localization(initial_pose, odom_alphas)

    gps     = GPS(robot, 2.0)

    #add the data callbacks to the localization modules
    sensors.addCallback(localization_sensors.receiveSensorData)
    sensors.addCallback(localization_gps_sensor.receiveSensorData)


    robot.addCallback(localization_sensors.receiveOdom)
    robot.addCallback(localization_gps.receiveOdom)
    robot.addCallback(localization_gps_sensor.receiveOdom)


    gps.addCallback(localization_gps.receiveGPS)
    gps.addCallback(localization_gps_sensor.receiveGPS)


    rate = rospy.Rate(25.0)
    robot.resetTime()

    data = {}

    data['ground_truth'] = {}
    data['ground_truth']['x'] = []
    data['ground_truth']['y'] = []

    #
    data['odom'] = {}
    data['odom']['x'] = []
    data['odom']['y'] = []

    #_
    data['kalman_gps'] = {}
    data['kalman_gps']['x'] = []
    data['kalman_gps']['y'] = []

    data['kalman_gps_sensors'] = {}
    data['kalman_gps_sensors']['x'] = []
    data['kalman_gps_sensors']['y'] = []


    data['kalman_sensors'] = {}
    data['kalman_sensors']['x'] = []
    data['kalman_sensors']['y'] = []

    
    while not rospy.is_shutdown():
        robot.run()

        #ground truth
        pose = robot.getGroundTruthPose()
        data['ground_truth']['x'].append(pose[0])
        data['ground_truth']['y'].append(pose[1])

        #
        pose = robot.getOdom()
        data['odom']['x'].append(pose[0])
        data['odom']['y'].append(pose[1])

        #
        pose = localization_gps.getPose()
        data['kalman_gps']['x'].append(pose[0])
        data['kalman_gps']['y'].append(pose[1])

        pose = localization_gps_sensor.getPose()
        data['kalman_gps_sensors']['x'].append(pose[0])
        data['kalman_gps_sensors']['y'].append(pose[1])

        pose = localization_sensors.getPose()
        data['kalman_sensors']['x'].append(pose[0])
        data['kalman_sensors']['y'].append(pose[1])

        if(robot.isEnded()):
            name = '~/.ros/test_eight_'+str(time.time())
            with open(name + '.pkl', 'wb') as f:
                pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)

            sub.Popen(('killall', 'roslaunch'))
            rospy.signal_shutdown('Quit')
            break


        #sensors.run()
        rate.sleep()