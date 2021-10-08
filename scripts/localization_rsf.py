#!/usr/bin/python3


import time
import math
from math import cos, sin, sqrt, log10, atan2
import numpy as np
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

import tf
from threading import Lock
from pylibrsf.pylibrsf import PyLibRSF


class LocalizationRSF:
    def __init__(self, initial_pose, wheel_baseline, odom_covariance, alg="gauss"):
        
        self.current_odom = None

        self.odom_pub = rospy.Publisher('localization_odom_rsf', Odometry, queue_size=10)

        #self.current_pose = np.matrix([[0],[0],[0]])
        self.sigma  = np.matrix(np.identity((3)))*100000.0
        self.current_odom_time = time.time()
        self.last_odom_time = self.current_odom_time
        self.initial_time = self.current_odom_time
        self.last_odom = np.matrix([[100.0],[100],[100]])
        self.odom_init = False
        self.rsf = PyLibRSF(alg)

        self.setInitialPose(initial_pose)
        self.wheel_baseline = wheel_baseline
        self.odom_covariance = [sqrt(odom_covariance[0]), sqrt(odom_covariance[1]), sqrt(odom_covariance[2])]
        
        



    def setInitialPose(self, pose):
        self.current_pose = np.matrix([[pose[0]],[pose[1]],[pose[2]]])
        self.last_odom = np.matrix([[pose[0]],[pose[1]],[pose[2]]])
        self.odom = np.matrix([[pose[0]],[pose[1]],[pose[2]]])
        self.last_odom_time = time.time()
        self.initial_time = self.last_odom_time
        
        timestamp = self.last_odom_time - self.initial_time
        self.rsf.setInitialPose(timestamp, pose[0], pose[1], pose[2], [100000000.0, 100000000., 0.001])


    def getVelocities(self, last_odom, new_odom, last_time, new_time):



        x = new_odom[0,0] - last_odom[0,0]
        y = new_odom[1,0] - last_odom[1,0]
        theta = last_odom[2,0]

        d = np.dot(np.matrix([[cos(theta), -sin(theta)],[sin(theta), cos(theta)]]).T, np.matrix([[x],[y]]))


        delta_time = new_time - last_time
        w = (new_odom[2,0] - last_odom[2,0])
        w = w - 2*math.pi * math.floor((w + math.pi) / (2*math.pi));
   
        return ((d[0,0], d[1,0]), w), delta_time

    

    def receiveOdom(self, odom):
       
        self.odom = odom
        self.odom_time = time.time()
        self.odom_init = True


    def receiveSensorData(self, sensor_data):
        #anchor_id, real_measurement, (x_s, y_s), self.l1, sensor_covariance
        if(not self.odom_init):
           return
           
        odom = self.odom
        new_odom = np.matrix([[odom[0]],[odom[1]],[odom[2]]])
        new_odom_time = self.odom_time
        #print(new_odom)
        velocities, delta_time = self.getVelocities(self.last_odom, new_odom, self.last_odom_time, new_odom_time)
       
        v_t, w_t = velocities
	
        if(delta_time > 0):
            v_l = (v_t[0] - self.wheel_baseline*w_t)/(delta_time)
            v_r = (v_t[0] + self.wheel_baseline*w_t)/(delta_time)
        
        #std::vector<vector<double>> &positions, std::vector<double> &range, std::vector<double> &L, std::vector<double> &covariance
        positions = []
        ranges = []
        Ls = []
        covariances = []
        for sensor in sensor_data:
           anchor_id, real_measurement, (x_s, y_s), l, sensor_covariance  = sensor
           positions.append([x_s, y_s])
           ranges.append(real_measurement)
           Ls.append(l)
           covariances.append(sqrt(sensor_covariance))
        
        
        timestampold = self.last_odom_time - self.initial_time
        timestamp = new_odom_time - self.initial_time
        #print('time ', timestamp)
        if(timestamp != timestampold):
            self.rsf.addStates(timestamp)
        
            self.rsf.addOdometry(timestamp, timestampold, [v_l, v_r, 0], self.wheel_baseline, self.odom_covariance)
        self.rsf.addMeasurement(timestamp, positions, ranges, Ls, covariances)
        
        #self.rsf.tuneErrorModel()        
        self.rsf.solve(timestamp, 60)
        
        self.current_pose = self.rsf.getState(timestamp)
        #print(timestamp, self.current_pose)
        #print(self.rsf.teste([4,5])) 
        #print(new_odom_time - self.initial_time, positions, Ls)
        
        self.last_odom = new_odom
        self.last_odom_time = new_odom_time
       



    def publishOdom(self):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world' # i.e. '/odom'
        

        msg.pose.pose.position = Point(self.current_pose[0, 0], self.current_pose[1, 0], 0.0)
        #print(msg.pose.pose.position)

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.current_pose[2,0])

        #type(pose) = geometry_msgs.msg.Pose
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        p_cov = np.array([0.0]*36).reshape(6,6)

        # position covariance
        p_cov[0:2,0:2] = self.sigma[0:2,0:2]
        # orientation covariance for Yaw
        # x and Yaw
        p_cov[5,0] = p_cov[0,5] = self.sigma[2,0]
        # y and Yaw
        p_cov[5,1] = p_cov[1,5] = self.sigma[2,1]
        # Yaw and Yaw
        p_cov[5,5] = self.sigma[2,2]

        msg.pose.covariance = tuple(p_cov.ravel().tolist())

        #print('publish')
        # Publish odometry message
        self.odom_pub.publish(msg)


    def getPose(self):
        return (self.current_pose[0], self.current_pose[1], self.current_pose[2])

