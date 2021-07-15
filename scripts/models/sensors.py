#!/usr/bin/python
import rospy
from scipy.optimize import linear_sum_assignment

import tf
import numpy as np
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
from rssi_model import RSSIModel
import time
import yaml
from numpy.linalg import cholesky
from models.decawave_model import DecawaveModel


class GPS:
    def __init__(self, robot, sigma):


        self.sigma = np.matrix([[sigma, 0],[0, sigma]])
        self.callback = []
        self.max_sense_time = 1.0
        self.robot = robot
        threading.Timer(0.1, self.run).start()



    def jacobian(self):
        def my_jacobian(x,y,theta):
            return np.matrix([[1,0,0],[0,1,0]])
        return my_jacobian

    def estimatedMeasurement(self):
        def my_measurment(x,y,theta):
            return np.matrix([[x], [y]])
        return my_measurment


    def addCallback(self, callback):
        self.callback.append(callback)

    def run(self):

        robot_pose = self.robot.getGroundTruthPose()
        x = robot_pose[0]
        y = robot_pose[1]


        covariance_matrix = self.sigma
        real_measurement = (np.matrix([x, y]) +  np.matrix(np.random.randn(2))*(cholesky(covariance_matrix).T)).T

        expected_measurement_function = self.estimatedMeasurement()# self.sensors[selected_sensor].getMeasurementRaw
        jacobian_function = self.jacobian()
        sensor_covariance = covariance_matrix


        for callback in self.callback:
            callback(real_measurement, expected_measurement_function, jacobian_function, sensor_covariance)


        if(not rospy.is_shutdown()):
            threading.Timer(self.max_sense_time, self.run).start() 

class DoubleSensors:
    def __init__(self, robot, sensors_file):
        self.robot = robot
        self.callback = []

        with open(sensors_file, 'r') as stream:
            self.sensors_config =  yaml.load(stream)


        self.correlation = self.sensors_config['params']['correlation']
        self.l1 = self.sensors_config['params']['l1']
        self.l2 = self.sensors_config['params']['l2']
        self.sensors = {}
        self.sense_time_ = 0;
        self.max_sense_time = float(self.sensors_config['params']['max_sense_time'])
        self.current_sensor_id = 0
        self.initSensors()
        threading.Timer(0.1, self.run).start()


    def initSensors(self):
        for sensor_id in self.sensors_config['sensors']:

           x = self.sensors_config['sensors'][sensor_id]['x']
           y = self.sensors_config['sensors'][sensor_id]['y']
           alpha = self.sensors_config['sensors'][sensor_id]['alpha']
           sigma = self.sensors_config['sensors'][sensor_id]['sigma']
           L0 = self.sensors_config['sensors'][sensor_id]['L0']
           d0 = self.sensors_config['sensors'][sensor_id]['d0']

           self.sensors[sensor_id] = RSSIModel(x, y, alpha, sigma, L0, d0)

    def jacobian(self, x_s, y_s, s_alpha_1, s_alpha_2,):
        def my_jacobian(x,y,theta):

            l1 = self.l1
            l2 = self.l2

            x_r1 = x + cos(theta)*l1
            y_r1 = y + sin(theta)*l1

            x_r2 = x + cos(theta)*l2
            y_r2 = y + sin(theta)*l2

            return np.matrix([[10*s_alpha_1*(x_r1-x_s)/((x_r1-x_s)**2 + (y_r1-y_s)**2), 10*s_alpha_1*(y_r1-y_s)/((x_r1-x_s)**2 + (y_r1-y_s)**2), 10*s_alpha_1*l1*((x_s-x)*sin(theta) - (y_s-y)*cos(theta))/((x_r1-x_s)**2 + (y_r1-y_s)**2)], \
                        [10*s_alpha_2*(x_r2-x_s)/((x_r2-x_s)**2 + (y_r2-y_s)**2), 10*s_alpha_2*(y_r2-y_s)/((x_r2-x_s)**2 + (y_r2-y_s)**2), 10*s_alpha_2*l2*((x_s-x)*sin(theta) - (y_s-y)*cos(theta))/((x_r2-x_s)**2 + (y_r2-y_s)**2)], \
                        ])
        return my_jacobian

    def estimatedMeasurement(self, sensor_id):
        def my_measurment(x,y,theta):

            l1 = self.l1
            l2 = self.l2

            x_r1 = x + cos(theta)*l1
            y_r1 = y + sin(theta)*l1

            x_r2 = x + cos(theta)*l2
            y_r2 = y + sin(theta)*l2


            measurment_func = self.sensors[sensor_id].getMeasurementRaw
            return np.matrix([[measurment_func(x_r1, y_r1)], [measurment_func(x_r2, y_r2)]])
        return my_measurment


    def selectSensor(self):
        #self.current_sensor_id = (self.current_sensor_id + 1)%len(self.sensors.keys())
        self.current_sensor_id = np.random.randint(len(self.sensors.keys()))
        return self.sensors.keys()[self.current_sensor_id]


    def addCallback(self, callback):
        self.callback.append(callback)

    
    def run(self):

        #if self.sense_time_ > time.time():
        #    return
        selected_sensor = self.selectSensor()

        #print(selected_sensor, time.time()- self.sense_time_)
        #new sense time
        self.sense_time_ = np.random.rand(1)[0]*self.max_sense_time + time.time()

        x_s = self.sensors[selected_sensor].x()
        y_s = self.sensors[selected_sensor].y()

        robot_pose = self.robot.getGroundTruthPose()
        #
        #
        #    reading sensor 1
        # 
        #
        #

        
        x_r1   = robot_pose[0] + cos(robot_pose[2])*self.l1
        y_r1   = robot_pose[1] + sin(robot_pose[2])*self.l1
        rssi_1 = self.sensors[selected_sensor].getMeasurementRaw(x_r1, y_r1)

        s_alpha_1 = self.sensors[selected_sensor].alpha()        
        s_sigma_1 = sqrt(self.sensors[selected_sensor].sigma())
        #s_sigma_1 = self.sensors[selected_sensor].sigma()

        measurement_function_1 = self.sensors[selected_sensor].getMeasurementRaw
        

        #
        #
        #    reading sensor 2
        # 
        #
        #

        x_r2   = robot_pose[0] + cos(robot_pose[2])*self.l2
        y_r2   = robot_pose[1] + sin(robot_pose[2])*self.l2
        rssi_2 = self.sensors[selected_sensor].getMeasurementRaw(x_r2, y_r2)

        s_alpha_2 = self.sensors[selected_sensor].alpha()
        s_sigma_2 = sqrt(self.sensors[selected_sensor].sigma())
        #s_sigma_2 = self.sensors[selected_sensor].sigma()

        measurement_function_2 = self.sensors[selected_sensor].getMeasurementRaw


        c = self.correlation
        covariance_matrix = np.matrix([[s_sigma_1**2,c*s_sigma_1*s_sigma_2],[c*s_sigma_1*s_sigma_2, s_sigma_2**2]])


        real_measurement = (np.matrix([rssi_1, rssi_2]) +  np.matrix(np.random.randn(2))*(cholesky(covariance_matrix).T)).T

        expected_measurement_function = self.estimatedMeasurement(selected_sensor)# self.sensors[selected_sensor].getMeasurementRaw
        jacobian_function = self.jacobian(x_s, y_s, s_alpha_1, s_alpha_2)
        sensor_covariance = covariance_matrix

        #print(x_s, y_s, rssi)
        for callback in self.callback:
            callback(real_measurement, expected_measurement_function, jacobian_function, sensor_covariance)

        if(not rospy.is_shutdown()):
            threading.Timer(self.max_sense_time, self.run).start()


class Sensors:
    def __init__(self, robot, sensors_file):
        self.robot = robot
        self.callback = []

        with open(sensors_file, 'r') as stream:
            self.sensors_config =  yaml.load(stream)


        self.sensors = {}
        self.sense_time_ = 0;
        self.max_sense_time = float(self.sensors_config['params']['max_sense_time'])
        self.current_sensor_id = 0
        self.initSensors()
        threading.Timer(0.1, self.run).start()


    def initSensors(self):
        for sensor_id in self.sensors_config['sensors']:

           x = self.sensors_config['sensors'][sensor_id]['x']
           y = self.sensors_config['sensors'][sensor_id]['y']
           alpha = self.sensors_config['sensors'][sensor_id]['alpha']
           sigma = self.sensors_config['sensors'][sensor_id]['sigma']
           L0 = self.sensors_config['sensors'][sensor_id]['L0']
           d0 = self.sensors_config['sensors'][sensor_id]['d0']

           self.sensors[sensor_id] = RSSIModel(x, y, alpha, sigma, L0, d0)

    def jacobian(self, x_s, y_s, s_alpha):
        def my_jacobian(x,y,theta):
            return np.matrix([10*s_alpha*(x-x_s)/((x-x_s)**2 + (y-y_s)**2), 10*s_alpha*(y-y_s)/((x-x_s)**2 + (y-y_s)**2), 0 ])
        return my_jacobian

    def estimatedMeasurement(self, sensor_id):
        def my_measurment(x,y,theta):
            measurment_func = self.sensors[sensor_id].getMeasurementRaw
            return np.matrix([[measurment_func(x, y)]])
        return my_measurment


    def selectSensor(self):
        #self.current_sensor_id = (self.current_sensor_id + 1)%len(self.sensors.keys())
        self.current_sensor_id = np.random.randint(len(self.sensors.keys()))
        return self.sensors.keys()[self.current_sensor_id]


    def addCallback(self, callback):
        self.callback.append(callback)

    
    def run(self):

        #if self.sense_time_ > time.time():
        #    return
        selected_sensor = self.selectSensor()

        #print(selected_sensor, time.time()- self.sense_time_)
        #new sense time
        self.sense_time_ = np.random.rand(1)[0]*self.max_sense_time + time.time()

        x_s = self.sensors[selected_sensor].x()
        y_s = self.sensors[selected_sensor].y()

        robot_pose = self.robot.getGroundTruthPose()
        s_alpha = self.sensors[selected_sensor].alpha()
        s_sigma = self.sensors[selected_sensor].sigma()


        real_measurement = self.sensors[selected_sensor].getMeasurement(robot_pose[0], robot_pose[1])

        expected_measurement_function = self.estimatedMeasurement(selected_sensor)# self.sensors[selected_sensor].getMeasurementRaw
        jacobian_function = self.jacobian(x_s, y_s, s_alpha)
        sensor_covariance = s_sigma
        #print(x_s, y_s, rssi)
        for callback in self.callback:
            callback(real_measurement, expected_measurement_function, jacobian_function, sensor_covariance)

        if(not rospy.is_shutdown()):
            threading.Timer(self.max_sense_time, self.run).start()



class Decawave:
    def __init__(self, robot, sensors_file):
        self.robot = robot
        self.callback = []

        with open(sensors_file, 'r') as stream:
            self.sensors_config =  yaml.load(stream)


        self.sensors = {}
        self.sense_time_ = 0;
        self.max_sense_time = float(self.sensors_config['params']['max_sense_time'])
        self.current_sensor_id = 0
        self.initSensors()
        threading.Timer(0.1, self.run).start()


    def initSensors(self):
        for sensor_id in self.sensors_config['sensors']:

           x = self.sensors_config['sensors'][sensor_id]['x']
           y = self.sensors_config['sensors'][sensor_id]['y']
           sigma = self.sensors_config['sensors'][sensor_id]['sigma']

           self.sensors[sensor_id] = DecawaveModel(x, y, sigma)


    def jacobian(self, x_s, y_s):
        def my_jacobian(x,y,theta):
            return np.matrix([(x-x_s)/sqrt((x-x_s)**2 + (y-y_s)**2), (y-y_s)/sqrt((x-x_s)**2 + (y-y_s)**2), 0 ])
        return my_jacobian

    def estimatedMeasurement(self, sensor_id):
        def my_measurment(x,y,theta):
            measurment_func = self.sensors[sensor_id].getMeasurementRaw
            return np.matrix([[measurment_func(x, y)]]);
        return my_measurment



    def selectSensor(self):
        #self.current_sensor_id = (self.current_sensor_id + 1)%len(self.sensors.keys())
        self.current_sensor_id = np.random.randint(len(self.sensors.keys()))
        return self.sensors.keys()[self.current_sensor_id]


    def addCallback(self, callback):
        self.callback.append(callback)

    def run(self):

        #if self.sense_time_ > time.time():
        #    return
        selected_sensor = self.selectSensor()

        #print(selected_sensor, time.time()- self.sense_time_)
        #new sense time
        self.sense_time_ = np.random.rand(1)[0]*self.max_sense_time + time.time()

        x_s = self.sensors[selected_sensor].x()
        y_s = self.sensors[selected_sensor].y()

        robot_pose = self.robot.getGroundTruthPose()
        s_sigma = self.sensors[selected_sensor].sigma()


        real_measurement = self.sensors[selected_sensor].getMeasurement(robot_pose[0], robot_pose[1])

        expected_measurement_function = self.estimatedMeasurement(selected_sensor)# self.sensors[selected_sensor].getMeasurementRaw
        jacobian_function = self.jacobian(x_s, y_s)
        sensor_covariance = s_sigma
        #print(x_s, y_s, rssi)
        for callback in self.callback:
            callback(real_measurement, expected_measurement_function, jacobian_function, sensor_covariance)

        if(not rospy.is_shutdown()):
            threading.Timer(self.max_sense_time, self.run).start()




class DoubleDecawave:
    def __init__(self, robot, sensors_file):
        self.robot = robot
        self.callback = []

        with open(sensors_file, 'r') as stream:
            self.sensors_config =  yaml.load(stream)


        self.correlation = self.sensors_config['params']['correlation']
        self.l1 = self.sensors_config['params']['l1']
        self.l2 = self.sensors_config['params']['l2']
        self.sensors = {}
        self.sense_time_ = 0;
        self.max_sense_time = float(self.sensors_config['params']['max_sense_time'])
        self.current_sensor_id = 0
        self.initSensors()
        threading.Timer(0.1, self.run).start()


    def initSensors(self):
        for sensor_id in self.sensors_config['sensors']:

           x = self.sensors_config['sensors'][sensor_id]['x']
           y = self.sensors_config['sensors'][sensor_id]['y']
           sigma = self.sensors_config['sensors'][sensor_id]['sigma']

           self.sensors[sensor_id] = DecawaveModel(x, y, sigma)

    def jacobian(self, x_s, y_s):
        def my_jacobian(x,y,theta):

            l1 = self.l1
            l2 = self.l2

            x_r1 = x + cos(theta)*l1
            y_r1 = y + sin(theta)*l1

            x_r2 = x + cos(theta)*l2
            y_r2 = y + sin(theta)*l2

            return np.matrix([[(x_r1-x_s)/sqrt((x_r1-x_s)**2 + (y_r1-y_s)**2), (y_r1-y_s)/sqrt((x_r1-x_s)**2 + (y_r1-y_s)**2), l1*((x_s-x)*sin(theta) - (y_s-y)*cos(theta))/sqrt((x_r1-x_s)**2 + (y_r1-y_s)**2)], \
                        [(x_r2-x_s)/sqrt((x_r2-x_s)**2 + (y_r2-y_s)**2), (y_r2-y_s)/sqrt((x_r2-x_s)**2 + (y_r2-y_s)**2), l2*((x_s-x)*sin(theta) - (y_s-y)*cos(theta))/sqrt((x_r2-x_s)**2 + (y_r2-y_s)**2)], \
                        ])
        return my_jacobian

    def estimatedMeasurement(self, sensor_id):
        def my_measurment(x,y,theta):

            l1 = self.l1
            l2 = self.l2

            x_r1 = x + cos(theta)*l1
            y_r1 = y + sin(theta)*l1

            x_r2 = x + cos(theta)*l2
            y_r2 = y + sin(theta)*l2


            measurment_func = self.sensors[sensor_id].getMeasurementRaw
            return np.matrix([[measurment_func(x_r1, y_r1)], [measurment_func(x_r2, y_r2)]])
        return my_measurment


    def selectSensor(self):
        #self.current_sensor_id = (self.current_sensor_id + 1)%len(self.sensors.keys())
        self.current_sensor_id = np.random.randint(len(self.sensors.keys()))
        return self.sensors.keys()[self.current_sensor_id]


    def addCallback(self, callback):
        self.callback.append(callback)

    
    
    def run(self):

        #if self.sense_time_ > time.time():
        #    return
        selected_sensor = self.selectSensor()

        #print(selected_sensor, time.time()- self.sense_time_)
        #new sense time
        self.sense_time_ = np.random.rand(1)[0]*self.max_sense_time + time.time()

        x_s = self.sensors[selected_sensor].x()
        y_s = self.sensors[selected_sensor].y()

        robot_pose = self.robot.getGroundTruthPose()
        #
        #
        #    reading sensor 1
        # 
        #
        #

        
        x_r1   = robot_pose[0] + cos(robot_pose[2])*self.l1
        y_r1   = robot_pose[1] + sin(robot_pose[2])*self.l1
        distance_1 = self.sensors[selected_sensor].getMeasurementRaw(x_r1, y_r1)

        s_sigma_1 = sqrt(self.sensors[selected_sensor].sigma())
        #s_sigma_1 = self.sensors[selected_sensor].sigma()

        measurement_function_1 = self.sensors[selected_sensor].getMeasurementRaw
        

        #
        #
        #    reading sensor 2
        # 
        #
        #

        x_r2   = robot_pose[0] + cos(robot_pose[2])*self.l2
        y_r2   = robot_pose[1] + sin(robot_pose[2])*self.l2
        distance_2 = self.sensors[selected_sensor].getMeasurementRaw(x_r2, y_r2)

        s_sigma_2 = sqrt(self.sensors[selected_sensor].sigma())
        #s_sigma_2 = self.sensors[selected_sensor].sigma()

        measurement_function_2 = self.sensors[selected_sensor].getMeasurementRaw


        c = self.correlation
        covariance_matrix = np.matrix([[s_sigma_1**2,c*s_sigma_1*s_sigma_2],[c*s_sigma_1*s_sigma_2, s_sigma_2**2]])
        
        #print(distance_1, distance_2)
        real_measurement = (np.matrix([distance_1, distance_2]) +  np.matrix(np.random.randn(2))*(cholesky(covariance_matrix).T)).T

        expected_measurement_function = self.estimatedMeasurement(selected_sensor)# self.sensors[selected_sensor].getMeasurementRaw
        jacobian_function = self.jacobian(x_s, y_s)
        sensor_covariance = covariance_matrix

        #print(x_s, y_s, rssi)
        for callback in self.callback:
            callback(real_measurement, expected_measurement_function, jacobian_function, sensor_covariance)

        if(not rospy.is_shutdown()):
            threading.Timer(self.max_sense_time, self.run).start()





