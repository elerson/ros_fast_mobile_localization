#!/usr/bin/python


import time
import math
from math import cos, sin, sqrt, log10, atan2
import numpy as np
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

import tf
from threading import Lock



class LocalizationEKF:
    def __init__(self, initial_pose, odom_alphas):
        
        self.current_odom = None

        self.odom_pub = rospy.Publisher('localization_odom_ekf', Odometry, queue_size=10)

        #self.current_pose = np.matrix([[0],[0],[0]])
        self.sigma  = np.matrix(np.identity((3)))*0.0000000001
        self.current_odom_time = time.time()
        self.current_odom = np.matrix([[0],[0],[0]])

        self.setInitialPose(initial_pose)

        self.alpha1 = odom_alphas[0]
        self.alpha2 = odom_alphas[1]
        self.alpha3 = odom_alphas[2]
        self.alpha4 = odom_alphas[3]
        self.kalman_mutex = Lock()


    def setInitialPose(self, pose):
        self.current_pose = np.matrix([[pose[0]],[pose[1]],[pose[2]]])
        self.current_odom = np.matrix([[pose[0]],[pose[1]],[pose[2]]])


    def getVelocities(self, last_odom, new_odom, last_time, new_time):

        x = new_odom[0,0] - last_odom[0,0]
        y = new_odom[1,0] - last_odom[1,0]

        direction =  cos(last_odom[2,0])*x + sin(last_odom[2,0])*y + 0.000000000001
        direction /= abs(direction) 


        delta_time = new_time - last_time
        if(delta_time == 0):
            return (0, 0.0), 0

        v = direction*math.sqrt(x**2 + y**2)/delta_time
        w = (new_odom[2,0] - last_odom[2,0])/delta_time

        return (v, w), delta_time


    def receiveOdom(self, odom):
        self.kalman_mutex.acquire()

        new_odom = np.matrix([[odom[0]],[odom[1]],[odom[2]]])
        new_odom_time = time.time()
        #print(new_odom)
        velocities, delta_time = self.getVelocities(self.current_odom, new_odom, self.current_odom_time, new_odom_time)
        #apply the prediction filter - kalman filter
        if(not (velocities[1] == 0.0)):
            #self.runKalmanFilter(velocities, delta_time, x_s, y_s, rssi, s_alpha, s_sigma, measurement_function)
            self.updatePrediction(velocities, delta_time)

        self.current_odom = new_odom
        self.current_odom_time = new_odom_time
        self.kalman_mutex.release()
        #print('odom')
        self.publishOdom()


    def receiveSensorData(self, real_measurement, expected_measurement_function, jacobian_function, sensor_covariance):
        self.kalman_mutex.acquire()
        #apply the mesurement filter - kalman filter
        self.updateMeasurementGeneric(real_measurement, expected_measurement_function, jacobian_function, sensor_covariance)
        self.kalman_mutex.release()


    def receiveDoubleSensorData(self, x_s, y_s, sensor_1_data, sensor_2_data, covariance_matrix):
        self.kalman_mutex.acquire()
        #apply the mesurement filter - kalman filter
        #TODO verify if both sensors are inside the measurement position
        distance = sqrt((self.current_pose[0, 0] - x_s)**2 + (self.current_pose[1, 0] - y_s)**2)
        if(distance > 1.0):
            self.updateDoubleMeasurement(x_s, y_s, sensor_1_data, sensor_2_data, covariance_matrix)
        self.kalman_mutex.release()

        self.publishOdom()
  
    def receiveGPS(self, x, y, sigma):
        self.kalman_mutex.acquire()
        self.updateGPSMeasurement(x, y, sigma)
        self.kalman_mutex.release()

        #self.current_pose  = np.matrix([[x], [y], [0]])
        self.publishOdom()


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
        return (self.current_pose[0,0], self.current_pose[1,0], self.current_pose[2,0])


    def updatePrediction(self, velocities, delta_time):
       
       
        #update using the robot dynamics
        theta = self.current_pose[2,0]
        v_t   = velocities[0]
        w_t   = velocities[1]

        G_t   = np.matrix([[1, 0, -(v_t/w_t)*cos(theta) + (v_t/w_t)*cos(theta + w_t*delta_time)], \
                           [0, 1, -(v_t/w_t)*sin(theta) + (v_t/w_t)*sin(theta + w_t*delta_time)], \
                           [0, 0, 1]])

        V_t   = np.matrix([[(-sin(theta) + sin(theta+w_t*delta_time))/w_t, v_t*(sin(theta) - sin(theta + w_t*delta_time))/w_t**2 + (v_t*cos(theta + w_t*delta_time)*delta_time)/w_t], \
                           [(cos(theta) - cos(theta+w_t*delta_time))/w_t, -v_t*(cos(theta) - cos(theta + w_t*delta_time))/w_t**2 + (v_t*sin(theta + w_t*delta_time)*delta_time)/w_t], \
                           [0, delta_time]])

        M_t   = np.matrix([[self.alpha1*(v_t**2)+self.alpha2*(w_t**2), 0], \
                           [0, self.alpha3*(v_t**2)+self.alpha4*(w_t**2)]])

        pose_ = self.current_pose + np.matrix([[-(v_t/w_t)*sin(theta) + (v_t/w_t)*sin(theta + w_t*delta_time)], \
                                               [(v_t/w_t)*cos(theta)  - (v_t/w_t)*cos(theta + w_t*delta_time)], \
                                               [w_t*delta_time]])
        
        sigma_= np.dot(np.dot(G_t,self.sigma), G_t.T)  + np.dot(np.dot(V_t,M_t), V_t.T)


        if pose_[2, 0] > 2*math.pi:
            pose_[2, 0]  -=  2*math.pi

        if pose_[2, 0] < -2*math.pi:
            pose_[2, 0]  +=  2*math.pi

        self.sigma = sigma_
        self.current_pose  = pose_
        
    def updateMeasurementGeneric(self, real_measurement, expected_measurement_function, jacobian_function, sensor_covariance):

        #print('measurement')
        sigma_ = self.sigma
        pose_  = self.current_pose
        #print(pose_)

         #sensor data
        z_s = real_measurement

        z_ = expected_measurement_function(pose_[0,0], pose_[1,0], pose_[2,0])

        s_sigma = sensor_covariance# np.matrix([[s_sigma_1,0], [0, s_sigma_2]])

        H_t = jacobian_function(pose_[0,0], pose_[1,0], pose_[2,0])

        Q_t = s_sigma #+ np.dot(H_t, np.dot(sigma_, H_t.T))

        S_t = np.dot(np.dot(H_t, sigma_), H_t.T) + Q_t

        K_t = np.dot(np.dot(sigma_, H_t.T),np.linalg.pinv(S_t))
        #print( np.dot(sigma_, H_t.T))
        pose_  = pose_ + np.dot(K_t,(z_s - z_))
        sigma_ = np.dot((np.matrix(np.eye(3)) - np.dot(K_t, H_t)), sigma_)

        #print(sigma_)

        if pose_[2, 0] > 2*math.pi:
            pose_[2, 0]  -=  2*math.pi

        if pose_[2, 0] < -2*math.pi:
            pose_[2, 0]  +=  2*math.pi

        
        self.sigma = sigma_
        self.current_pose  = pose_ 
