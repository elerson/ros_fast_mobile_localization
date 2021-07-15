#!/usr/bin/python
import rospy
import tf
import numpy as np
import math
from math import cos, sin, log10, sqrt, atan2
from collections import deque


#ROS Imports
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
from ros_decawave.msg import AnchorArray

import time
import yaml
import utm

from visualization_msgs.msg import Marker
   

class Robot:
    def __init__(self, devices):


        self.ground_truth = (0, 0, 0)

        

        (dev1_name, dev_1_distance), (dev2_name, dev_2_distance) = devices

        self.anchors_list = set([])
        self.l1 = dev_1_distance
        self.l2 = dev_2_distance

        self.data = np.empty(shape=[0, 2])
        self.tf_listener = tf.TransformListener()

        self.initializeQueues()
        self.marker_arrow_pub  = rospy.Publisher('sensor_arrow_marker', Marker, queue_size=10)
        #rospy.Subscriber(dev1_name, AnchorArray, self.decawaveMarker)
        rospy.Subscriber(dev1_name, AnchorArray, self.decawaveSensor1)
        rospy.Subscriber(dev2_name, AnchorArray, self.decawaveSensor2)

        rospy.Subscriber("/pose", PoseStamped, self.getData)



    def nodePosition(self, anchor_id):
        (trans,rot) = self.tf_listener.lookupTransform('world', anchor_id, rospy.Time(0))
        #print('teste', trans)
        return trans[0], trans[1], trans[2]

    def initializeQueues(self):
        self.sensor_1 = {}
        self.sensor_2 = {}
        for anchor in self.anchors_list:
            self.sensor_1[anchor] = deque()
            self.sensor_2[anchor] = deque()


    def getData(self, data):

        (trans,rot) = self.tf_listener.lookupTransform("/world", "/base_link", rospy.Time(0))


        orientation = tf.transformations.euler_from_quaternion(rot)
        self.ground_truth = (trans[0], trans[1], trans[2] , orientation[2])

        for anchor_id in self.anchors_list:

            if(anchor_id in self.sensor_1 and anchor_id in self.sensor_2 and len(self.sensor_1[anchor_id]) >= 1 and len(self.sensor_2[anchor_id]) >= 1):
                    distance_1, time_1 = self.sensor_1[anchor_id].popleft()
                    distance_2, time_2 = self.sensor_2[anchor_id].popleft()


                    x_s, y_s, z_s = self.nodePosition(anchor_id)
  

                    mesurement_func_1 = self.estimatedMeasurement(x_s, y_s, z_s, self.l1)# self.sensors[selected_sensor].getMeasurementRaw
                    mesurement_func_2 = self.estimatedMeasurement(x_s, y_s, z_s, self.l2)

                    mesurement_1 = mesurement_func_1(self.ground_truth[0], self.ground_truth[1], self.ground_truth[2], self.ground_truth[3])
                    mesurement_2 = mesurement_func_2(self.ground_truth[0], self.ground_truth[1], self.ground_truth[2], self.ground_truth[3])

                    #print( distance_2 - mesurement_2, distance_1 - mesurement_1)


                    self.data = np.append(self.data, [[mesurement_1 - distance_1, mesurement_2 - distance_2]], axis=0)
                   
                    self.sensor_1[anchor_id].clear()
                    self.sensor_2[anchor_id].clear()

    def getDistance(self, x, y, z):
        return sqrt(x**2 + y**2 + z**2)

    def estimatedMeasurement(self, x_s, y_s, z_s, l):

        def my_measurment(x,y,z,theta):

            x_r1 = x + cos(theta)*l
            y_r1 = y + sin(theta)*l
            z_r1 = z

            return self.getDistance(x_s-x_r1 , y_s-y_r1, z_s-z_r1)
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

        

    def run(self):
        #print(self.data)
        print (np.cov(np.matrix(self.data).T))
        pass



if __name__ == "__main__":

    rospy.init_node('decawaveparams', anonymous=True)


    devices = [('/dwc0af/tag_status', -0.225), ('/dwc4b8/tag_status', 0.225)]

    robot = Robot(devices)

    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        
        robot.run()

        rate.sleep()