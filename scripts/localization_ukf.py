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



class LocalizationUKF:
    def __init__(self, initial_pose, odom_alphas, k_ = 0.0, alpha_ = 0.7, beta_ = 2.0):
        
        self.current_odom = None

        self.odom_pub = rospy.Publisher('localization_odom_ukf', Odometry, queue_size=10)

        #self.current_pose = np.matrix([[0],[0],[0]])
        self.sigma  = np.matrix(np.identity((3)))*100000.0
        self.current_odom_time = time.time()
        self.current_odom = np.matrix([[0],[0],[0]])

        self.setInitialPose(initial_pose)

        self.alpha1 = odom_alphas[0]
        self.alpha2 = odom_alphas[1]
        self.alpha3 = odom_alphas[2]
        self.alpha4 = odom_alphas[3]
        self.kalman_mutex = Lock()

        #definig the weights for the UKF
        n = max(self.sigma.shape[0], self.sigma.shape[1])
        lambda_ = (alpha_**2)*(n + k_) - n

        self.n = n
        self.W_m = [lambda_/(n + lambda_)]
        for i in range(2*n):
            self.W_m.append(1/(2*(n + lambda_)))

        self.W_c = [lambda_/(n + lambda_) + (1-(alpha_**2) + beta_)]
        for i in range(2*n):
            self.W_c.append(1/(2*(n + lambda_)))

        self.lambda_ = lambda_
        self.alpha_  = alpha_
        self.beta_   = beta_ 
        self.k_      = k_
        # n = 2

        n = 2
        lambda_ = (alpha_**2)*(n + k_) - n
        self.W_m2 = [lambda_/(n + lambda_)]
        for i in range(2*n):
            self.W_m2.append(1/(2*(n + lambda_)))


        self.W_c2 = [lambda_/(n + lambda_) + (1-(alpha_**2) + beta_)]
        for i in range(2*n):
            self.W_c2.append(1/(2*(n + lambda_)))







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


    def updatePrediction2(self, velocities, delta_time):
       
       
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


    def updatePrediction(self, velocities, delta_time):
       
       
        #update using the robot dynamics
        theta = self.current_pose[2,0]
        v_t   = velocities[0]
        w_t   = velocities[1]


        # V_t   = np.matrix([[(-sin(theta) + sin(theta+w_t*delta_time))/w_t, v_t*(sin(theta) - sin(theta + w_t*delta_time))/w_t**2 + (v_t*cos(theta + w_t*delta_time)*delta_time)/w_t], \
        #                   [(cos(theta) - cos(theta+w_t*delta_time))/w_t, -v_t*(cos(theta) - cos(theta + w_t*delta_time))/w_t**2 + (v_t*sin(theta + w_t*delta_time)*delta_time)/w_t], \
        #                   [0, delta_time]])

        M_t   = np.matrix([[self.alpha1*(v_t**2)+self.alpha2*(w_t**2), 0], \
                           [0, self.alpha3*(v_t**2)+self.alpha4*(w_t**2)]])

        

        #1) defining the sigma points
        points_ = self.getSigmaPoints(self.current_pose, self.sigma, self.lambda_)

        #2) propagate the sigma points with the system dynamics
        Y = []
        for i in range(1+2*self.n):
            theta     = points_[i][2,0]
            new_point = points_[i] + np.matrix([[-(v_t/w_t)*sin(theta) + (v_t/w_t)*sin(theta + w_t*delta_time)], \
                                               [(v_t/w_t)*cos(theta)  - (v_t/w_t)*cos(theta + w_t*delta_time)], \
                                               [w_t*delta_time]])
            Y.append(new_point)

        theta = self.current_pose[2,0]


        #3 compute the predicted mean "pose_" and convariance "sigma_"
        pose_ = np.matrix([[0.0],[0.0],[0.0]])

        for i in range(1+2*self.n):
            #print(self.W_m[i], Y[i])
            pose_ += self.W_m[i]*Y[i]


        sigma_ = np.matrix(np.zeros(self.sigma.shape))
        for i in range(1+2*self.n):
            diff_ =(Y[i] - pose_)
            sigma_ += self.W_c[i]*np.dot(diff_, diff_.T)


        #4 caculate the error caused by the error in v and w
        n_ = 2
        lambda_ = (self.alpha_**2)*(n_ + self.k_) - n_
        sigma_points_v = self.getSigmaPoints(np.matrix([[v_t],[w_t]]), M_t, lambda_)
        poses_v = []
        for sigma_v in sigma_points_v:
            v_t = sigma_v[0, 0]
            w_t = sigma_v[1, 0]

            new_point = self.current_pose + np.matrix([[-(v_t/w_t)*sin(theta) + (v_t/w_t)*sin(theta + w_t*delta_time)], \
                                               [(v_t/w_t)*cos(theta)  - (v_t/w_t)*cos(theta + w_t*delta_time)], \
                                               [w_t*delta_time]])

            poses_v.append(new_point)

        pose_v_m = np.matrix([[0.0],[0.0],[0.0]])
        
        for i in range(1+2*n_):
            pose_v_m += self.W_m2[i]*poses_v[i]


        sigma_v = np.matrix(np.zeros(sigma_.shape))
        for i in range(1+2*n_):
            diff_ = (poses_v[i] - pose_v_m)
            sigma_v += self.W_c2[i]*np.dot(diff_, diff_.T)


        sigma_ += sigma_v


        if pose_[2, 0] > 2*math.pi:
            pose_[2, 0]  -=  2*math.pi

        if pose_[2, 0] < -2*math.pi:
            pose_[2, 0]  +=  2*math.pi

        self.sigma = sigma_
        self.current_pose  = pose_

    def getSigmaPoints(self, m, P, lambda_):

        #defining the covariance square
        u, s, vh = np.linalg.svd(P, full_matrices=True)
        s = np.sqrt(np.diag(s))
        P_square = np.dot(u, np.dot(s, vh))

        n_ = m.shape[0]
        #print(m, P_square, n_+lambda_)

        
        #1) defining the sigma points
        points_ = [m]
        for i in range(n_):
            points_.append(m + np.matrix(sqrt(n_+lambda_)*P_square[:,i]))

        for i in range(n_):
            points_.append(m - np.matrix(sqrt(n_+lambda_)*P_square[:,i]))

        return points_


        
    def updateMeasurementGeneric(self, real_measurement, expected_measurement_function, jacobian_function, sensor_covariance):

        #print('localization')
        sigma_ = self.sigma
        pose_  = self.current_pose
        #print("pose", pose_)


        #1) defining the sigma points
        points_ = self.getSigmaPoints(pose_, sigma_, self.lambda_)


        #2) propagate the sigma points with measurement model
        Y = []
        #print("size", len(points_), 2*self.n)
        for i in range(1+2*self.n):
            #print('points', points_[i])
            new_point = expected_measurement_function(points_[i][0,0], points_[i][1,0], points_[i][2,0])
            #print(new_point.shape)
            Y.append(new_point)


        #print('self.n', self.n)
        #3 compute the predicted mean "pose_" and convariance "sigma_"
        mu_ = np.matrix(np.zeros(Y[0].shape))
        for i in range(1+2*self.n):
            #print(self.W_m[i]*Y[i], mu_)
            mu_ += self.W_m[i]*Y[i]




        S_t = np.matrix(np.zeros((mu_.shape[0],mu_.shape[0])))
        for i in range(1+2*self.n):
            diff_ =(Y[i] - mu_)
            #print("mu", diff_, S_t)
            S_t += self.W_c[i]*np.dot(diff_, diff_.T)

        S_t +=  sensor_covariance


        C_t = np.matrix(np.zeros((self.sigma.shape[0], mu_.shape[0])))
        for i in range(1+2*self.n):
            #print('pose', points_[i],  pose_)
            C_t += self.W_c[i]*np.dot((points_[i] - pose_),(Y[i] - mu_).T)


        #print(pose_)

         #sensor data
        z_s = real_measurement

        K_t = np.dot(C_t,np.linalg.pinv(S_t))
        #print('pose 1', pose_)
        pose_  = pose_ + np.dot(K_t, (z_s - mu_))
        #print('pose 2', z_s)
        sigma_ = sigma_ - np.dot(K_t, np.dot(S_t, K_t.T))


        #print(sigma_)

        if pose_[2, 0] > 2*math.pi:
            pose_[2, 0]  -=  2*math.pi

        if pose_[2, 0] < -2*math.pi:
            pose_[2, 0]  +=  2*math.pi

        #print(pose_)
        self.sigma = sigma_
        self.current_pose  = pose_ 
