#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date  : 12/06/2023
Update: 
- Thêm chế độ thực nghiệm điểm thao tác.
"""
import roslib

import sys
import time
from decimal import *
import math
import rospy

# -- add 19/01/2022
import subprocess
import re
import os

from sti_msgs.msg import *

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int8, Bool
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees
from sick_lidar_localization.msg import LocalizationControllerResultMessage0502

#--------------------------------------------------------------------------------- ROS
class ros_control():
    def __init__(self):
        rospy.init_node('robot_pose', anonymous=False)
        self.rate = rospy.Rate(30)
        
        self.pub_pose = rospy.Publisher("/robotPose_lidarLOC", PoseStamped, queue_size= 20)
        self.data_pose = PoseStamped()

        # -- Data LidaLoc
        rospy.Subscriber("/localizationcontroller/out/localizationcontroller_result_message_0502", LocalizationControllerResultMessage0502, self.lidarLoc_callback) 
        self.lidarLoc_data = LocalizationControllerResultMessage0502()


    def callback_getPose(self, dat):
        self.robotPose_lidarLOC = dat
        # doi quaternion -> rad    
        quaternion1 = (dat.pose.orientation.x, dat.pose.orientation.y,\
                    dat.pose.orientation.z, dat.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion1)

        self.NN_infoRespond.x = round(dat.pose.position.x, 3)
        self.NN_infoRespond.y = round(dat.pose.position.y, 3)
        self.NN_infoRespond.z = round(euler[2], 3)

    def lidarLoc_callback(self, data):
        self.lidarLoc_data = data
        self.data_pose.header = self.lidarLoc_data.header

        self.data_pose.pose.position.x = self.lidarLoc_data.x/1000.
        self.data_pose.pose.position.y = self.lidarLoc_data.y/1000.

        qua = tf.transformations.quaternion_from_euler(0. , 0., radians(self.lidarLoc_data.heading/1000.))
        self.data_pose.pose.orientation.z = qua[2]
        self.data_pose.pose.orientation.w = qua[3]

        self.pub_pose.publish(self.data_pose)


    def run(self):
        self.rate.sleep()

def main():
    # Start the job threads
    class_1 = ros_control()
    # Keep the main thread running, otherwise signals are ignored.
    while not rospy.is_shutdown():
        class_1.run()

if __name__ == '__main__':
	main()

"""
Stt :
0: chờ đủ dữ liệu để parking
1: chờ tín hiệu parking
21:  tính khoảng cách tiến lùi
-21: thực hiện di chuyen tiến lùi
-210: thực hiện quay trước nếu gặp TH AGV bị lệch góc lớn
31: tính góc quay để lùi vào kệ
-31: thực hiện quay
41: Parking
51: completed - Đợi Reset
52: error: bien doi tf loi

"""
