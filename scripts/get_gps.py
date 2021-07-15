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


from nav_msgs.msg import MapMetaData
from models.rssi_model import RSSIModel
from localization import Localization
import time
import yaml
import utm

   

class Robot:
    def __init__(self):
        self.n = 0
        self.latitude = 0
        self.longitude = 0
        rospy.Subscriber("/gps", Gps_msg, self.getGPS)


   

    def getGPS(self, data):
        self.latitude  += data.latitude
        self.longitude += data.longitude
        self.n += 1
        print(self.latitude/self.n, self.longitude/self.n)


        #print(self.initial_pose, data.latitude, data.longitude)

    

    

if __name__ == "__main__":

    rospy.init_node('get gps', anonymous=True)

    robot = Robot()


    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():
  
        #sensors.run()
        rate.sleep()