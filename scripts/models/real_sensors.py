import tf

from ros_decawave.msg import AnchorArray
from collections import deque
import rospy
import numpy as np
import time
from math import cos, sin, sqrt, atan2
from ros_decawave.msg import Tag


class DoubleDecawaveGroundTruth:
    def __init__(self, devices):
        (dev1_name, dev_1_distance), (dev2_name, dev_2_distance) = devices

        self.l1 = dev_1_distance
        self.l2 = dev_2_distance

        rospy.Subscriber(dev1_name, Tag, self.decawaveSensor1)
        rospy.Subscriber(dev2_name, Tag, self.decawaveSensor2)

        self.sensor_1   =  (0,0,0)
        self.sensor_2   =  (0,0,0)
        self.pose       =  (0,0,0)



    def decawaveSensor1(self, data):
        self.sensor_1 = (data.x, data.y, data.z)

    def decawaveSensor2(self, data):
        self.sensor_2 = (data.x, data.y, data.z)

    def getPose(self):

        x_ = (self.sensor_1[0] + self.sensor_2[0])/2.0
        y_ = (self.sensor_1[1] + self.sensor_2[1])/2.0


        x_diff = (self.sensor_2[0] - self.sensor_1[0])
        y_diff = (self.sensor_2[1] - self.sensor_1[1])

        #print('angle', atan2(y_diff, x_diff))

        return (x_, y_, atan2(y_diff, x_diff))



class DoubleDecawaveReal:
    def __init__(self, devices, covariance_matrix):
        self.callback = []

        (dev1_name, dev_1_distance), (dev2_name, dev_2_distance) = devices

        self.anchors_list = set([])

        self.l1 = dev_1_distance
        self.l2 = dev_2_distance

        self.covariance_matrix = covariance_matrix
        self.tf_listener = tf.TransformListener()
        self.initializeQueues()

        rospy.Subscriber(dev1_name, AnchorArray, self.decawaveSensor1)
        rospy.Subscriber(dev2_name, AnchorArray, self.decawaveSensor2)


    def addCallback(self, callback):
        self.callback = []
        self.callback.append(callback)

    def initializeQueues(self):
        self.sensor_1 = {}
        self.sensor_2 = {}
        for anchor in self.anchors_list:
            self.sensor_1[anchor] = deque()
            self.sensor_2[anchor] = deque()
    
    
    def nodePosition(self, anchor_id):
        (trans,rot) = self.tf_listener.lookupTransform('world', anchor_id, rospy.Time(0))
        return trans[0], trans[1]


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

    def getDistance(self, x, y):
        return sqrt(x**2 + y**2)

    def estimatedMeasurement(self, x_s, y_s):

        def my_measurment(x,y,theta):

            l1 = self.l1
            l2 = self.l2

            x_r1 = x + cos(theta)*l1
            y_r1 = y + sin(theta)*l1

            x_r2 = x + cos(theta)*l2
            y_r2 = y + sin(theta)*l2


            return np.matrix([[self.getDistance(x_s-x_r1 , y_s-y_r1)], [self.getDistance(x_s -x_r2 , y_s - y_r2)]])
        return my_measurment



    def decawaveSensor1(self, data):

        for anchor in data.anchors:
            id_ = anchor.header.frame_id

            self.anchors_list = self.anchors_list.union(set([id_]))

            if(not id_ in self.sensor_2):
                self.sensor_2[id_] = deque()
            if(not id_ in self.sensor_1):
                self.sensor_1[id_] = deque()

            self.sensor_1[id_].append((anchor.distance, time.time()))

        #self.processQueues()

    def decawaveSensor2(self, data):

        for anchor in data.anchors:
            id_ = anchor.header.frame_id

            self.anchors_list = self.anchors_list.union(set([id_]))

            if(not id_ in self.sensor_2):
                self.sensor_2[id_] = deque()
            if(not id_ in self.sensor_1):
                self.sensor_1[id_] = deque()

            self.sensor_2[id_].append((anchor.distance, time.time()))

        self.processQueues()


    def processQueues(self):

        for anchor_id in self.anchors_list:
            if(len(self.sensor_1[anchor_id]) >= 1 and len(self.sensor_2[anchor_id]) >= 1):
                distance_1, time_1 = self.sensor_1[anchor_id].popleft()
                distance_2, time_2 = self.sensor_2[anchor_id].popleft()

                x_s, y_s = self.nodePosition(anchor_id)
                
                real_measurement = np.matrix([[distance_1], [distance_2]])

                expected_measurement_function = self.estimatedMeasurement(x_s, y_s)# self.sensors[selected_sensor].getMeasurementRaw
                jacobian_function = self.jacobian(x_s, y_s)
                sensor_covariance = self.covariance_matrix

                #print(x_s, y_s, rssi)
                for callback in self.callback:
                    callback(real_measurement, expected_measurement_function, jacobian_function, sensor_covariance)

                self.sensor_1[anchor_id].clear()
                self.sensor_2[anchor_id].clear()


class DecawaveReal:
    def __init__(self, devices, covariance_matrix, sensor_z=0.45):
        self.callback = []

        (dev1_name, dev_1_distance) = devices

        self.anchors_list = set([])

        self.sensor_z = sensor_z
        self.l1 = dev_1_distance
       
        self.covariance_matrix = covariance_matrix
        self.tf_listener = tf.TransformListener()
        self.initializeQueues()

        rospy.Subscriber(dev1_name, AnchorArray, self.decawaveSensor1)



    def addCallback(self, callback):
        self.callback = []
        self.callback.append(callback)

    def initializeQueues(self):
        self.sensor_1 = {}        
        for anchor in self.anchors_list:
            self.sensor_1[anchor] = deque()
    
    def nodePosition(self, anchor_id):
        try:
          (trans,rot) = self.tf_listener.lookupTransform('world', anchor_id, rospy.Time(0))
          return trans[0], trans[1], trans[2]
        except:
          return None, None


    def jacobian(self, x_s, y_s, z_s):
        def my_jacobian(x,y,theta):

            l1 = self.l1

            x_r1 = x + cos(theta)*l1
            y_r1 = y + sin(theta)*l1


            return np.matrix([[(x_r1-x_s)/sqrt((x_r1-x_s)**2 + (y_r1-y_s)**2 + (z_s-self.sensor_z)**2), (y_r1-y_s)/sqrt((x_r1-x_s)**2 + (y_r1-y_s)**2 + (z_s-self.sensor_z)**2), l1*((x_s-x)*sin(theta) - (y_s-y)*cos(theta))/sqrt((x_r1-x_s)**2 + (y_r1-y_s)**2 + (z_s-self.sensor_z)**2) ]])
        return my_jacobian

    def getDistance(self, x, y, z):
        return sqrt(x**2 + y**2 + z**2)

    def estimatedMeasurement(self, x_s, y_s, z_s):

        def my_measurment(x,y,theta):

            l1 = self.l1

            x_r1 = x + cos(theta)*l1
            y_r1 = y + sin(theta)*l1

            return np.matrix([[self.getDistance(x_s-x_r1 , y_s-y_r1, z_s-self.sensor_z)]])
        return my_measurment



    def decawaveSensor1(self, data):

        for anchor in data.anchors:
            id_ = anchor.header.frame_id

            self.anchors_list = self.anchors_list.union(set([id_]))

            if(not id_ in self.sensor_1):
                self.sensor_1[id_] = deque()

            self.sensor_1[id_].append((anchor.distance, time.time()))

        self.processQueues()

    def processQueues(self):

        for anchor_id in self.anchors_list:
            if(len(self.sensor_1[anchor_id]) >= 1):
                distance_1, time_1 = self.sensor_1[anchor_id].popleft()
                
                x_s, y_s, z_s = self.nodePosition(anchor_id)
                if(x_s == None):
                    continue
                
                real_measurement = np.matrix([[distance_1]])

                expected_measurement_function = self.estimatedMeasurement(x_s, y_s, z_s)# self.sensors[selected_sensor].getMeasurementRaw
                jacobian_function = self.jacobian(x_s, y_s, z_s)
                sensor_covariance = self.covariance_matrix

                #print(x_s, y_s, rssi)
                for callback in self.callback:
                    callback(real_measurement, expected_measurement_function, jacobian_function, sensor_covariance)

                self.sensor_1[anchor_id].clear()
                

class DecawaveRSF:
    def __init__(self, devices, covariance_matrix):
        self.callback = []

        (dev1_name, dev_1_distance) = devices

        self.anchors_list = set([])

        self.l1 = dev_1_distance
       
        self.covariance_matrix = covariance_matrix
        self.tf_listener = tf.TransformListener()
        self.initializeQueues()

        rospy.Subscriber(dev1_name, AnchorArray, self.decawaveSensor1)


    def addCallback(self, callback):
        self.callback = []
        self.callback.append(callback)

    def initializeQueues(self):
        self.sensor_1 = {}        
        for anchor in self.anchors_list:
            self.sensor_1[anchor] = deque()
    
    def nodePosition(self, anchor_id):
        try:
          (trans,rot) = self.tf_listener.lookupTransform('world', anchor_id, rospy.Time(0))
          return trans[0], trans[1]
        except:
          return None, None

    def decawaveSensor1(self, data):
        sensor_info = []
        for anchor in data.anchors:
            id_ = anchor.header.frame_id

            x_s, y_s = self.nodePosition(id_)
            
            if(x_s == None):
                continue
            
            sensor_info.append((id_, anchor.distance, (x_s, y_s), self.l1, self.covariance_matrix))

        for callback in self.callback:
            callback(sensor_info)


