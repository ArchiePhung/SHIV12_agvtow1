#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dev: Archie Phung
Date Modify: 16/8/2024

Advantage:
  From tf coef -> position tolerance < 1 cm

Function:
  - Spin AGV to precicise angle with odometry data
  - Exec command from keyboard
  - Intergrate with remote function of AGV

 >>> Odometry data
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance

>> sick odom data

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint64 telegram_count
uint64 timestamp
int32 source_id
int64 x_position
int64 y_position
int64 heading
uint32 sync_timestamp_sec
uint32 sync_timestamp_nsec
uint32 sync_timestamp_valid

"""

import rospy
from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs, acos, degrees

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Twist, PoseWithCovarianceStamped

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from message_pkg.msg import *
from sti_msgs.msg import *
from ros_canBus.msg import *
from sick_lidar_localization.msg import OdometryMessage0105, LocalizationControllerResultMessage0502

import os
import time

class ConvertMSG():
    def __init__(self):
        print("ROS Initial: Convert_odom_to_Sickodom105 !")
        rospy.init_node('convert_odom_to_sickodom105', anonymous = False, disable_signals=True) # False

        # -- SUBSCRIBER NODE -- 
        rospy.Subscriber("/localizationcontroller/out/odometry_message_0105", OdometryMessage0105, self.callback_Odometry) # or odom_combined
        self.odom_data = OdometryMessage0105()

        # -- PUBLISH TOPIC -- 
        self.pub_sick_odom = rospy.Publisher("/odom_lidarLoc", Odometry, queue_size=10)

        self.telegramCount = 0

        rospy.spin()

    # convert yaw, pitch, roll to quaternion
    def toQuaternion(self, yaw, pitch, roll): # yaw (Z), pitch (Y), roll (X)
        # Abbreviations for the various angular functions
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        # Quaternion q;
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return w, x, y, z

    ############################################ CALLBACK FUNCTION ########################################################################3
    def callback_Odometry(self, data):
        self.odom_data = data

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.seq = self.telegramCount

        # odom_msg.header.frame_id = 'odom'
        # odom_msg.child_frame_id = "base_link_lidarLoc"


        odom_msg.pose.covariance[0] = 0.001
        odom_msg.pose.covariance[7] = 0.001
        odom_msg.pose.covariance[35] = 0.001

        odom_msg.twist.covariance[0] = 0.0001
        odom_msg.twist.covariance[7] = 0.0001
        odom_msg.twist.covariance[35] = 0.0001

        # odom_msg.twist.twist.linear.x = 0.
        # odom_msg.twist.twist.linear.y = 0.
        # odom_msg.twist.twist.angular.z = 0.
        odom_msg.pose.pose.position.x = self.odom_data.x_position / 1000
        odom_msg.pose.pose.position.y = self.odom_data.y_position / 1000

        heading = self.odom_data.heading

        odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z = self.toQuaternion(PI * (heading / 1000.0) / 180.0, 0, 0)
        self.pub_sick_odom.publish(odom_msg)

        # Increment telegram counter
        self.telegramCount = self.telegramCount + 1

def main():
	print('Starting main program')
	program = ConvertMSG()
	print('Exiting main program')	

if __name__ == '__main__':
    main()
    