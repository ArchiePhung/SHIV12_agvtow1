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
from geometry_msgs.msg import Pose, PoseStamped, Twist, PoseWithCovarianceStamped, TwistWithCovarianceStamped

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
        print("ROS Initial: Convert_odom_to_Sickodom !")
        rospy.init_node('convert_odom_to_sickodom', anonymous = False, disable_signals=True) # False

        self.rate = rospy.Rate(40)

        # self.tolerance_rot_step1 = rospy.get_param('~tolerance_rot_step1',0.02)
        # self.vel_rot_step1 = rospy.get_param('~vel_rot_step1',0.32)      # 0.45
        self.vel_rot_step1 = 0.2

        # -- SUBSCRIBER NODE -- 
        # -- Keyboard command
        rospy.Subscriber("/raw_odom", Odometry, self.callback_Odometry) # or odom_combined
        self.poseRbMa_odom = Pose()
        self.twistRbMa_odom = Twist()
        self.is_recv_odom = False

        # rospy.Subscriber("/raw_vel", TwistWithCovarianceStamped, self.callback_Rawvel) # or odom_combined
        # self.rawvel_data = Twist()
        # self.is_recv_rawvel = False

#         rospy.Subscriber("/localizationcontroller/out/localizationcontroller_result_message_0502", LocalizationControllerResultMessage0502
# , self.callback_lls) # 
#         self.lls_data = LocalizationControllerResultMessage0502()
#         self.is_recv_lls = False

        # -- PUBLISH TOPIC -- 
        self.pub_sick_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.sick_odom_data = Odometry()
        
        # -- CONST --
        self.rate_sickOdom = 40
        self.ID_SENDER = 31

        self.time_tr = rospy.get_time()
        self.is_exit = 0
        self.theta_rb_odom = 0.0

        self.telegram_count = 0

    ############################################ CALLBACK FUNCTION ########################################################################3
    def callback_Odometry(self, data):
        self.twistRbMa_odom = data.twist.twist
        self.poseRbMa_odom = data.pose.pose
        # print(self.poseRbMa_odom)
        quata = ( self.poseRbMa_odom.orientation.x,\
                self.poseRbMa_odom.orientation.y,\
                self.poseRbMa_odom.orientation.z,\
                self.poseRbMa_odom.orientation.w )
        euler = euler_from_quaternion(quata)

        self.theta_rb_odom = euler[2]
        self.is_recv_odom = True
        # if self.theta_rb_odom <= 0:
        #     self.theta_rb_odom = self.theta_rb_odom + 2*PI

    # def callback_Rawvel(self, data):
    #     self.rawvel_data = data.twist.twist
    #     self.is_recv_rawvel = True

    # def callback_lls(self, data):
    #     self.is_recv_lls = True
    #     self.lls_data = data

    ############################################ DEF FUNCTION ########################################################################
    def toQuaternion(yaw, pitch, roll): # yaw (Z), pitch (Y), roll (X)
        # Abbreviations for the various angular functions
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        # Quaternion q;
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return w, x, y, z

    def sendOdometryTelegram(self, pub, vx, vy, omega, x, y, rw, rx, ry, rz):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.rostime.Time.now()
        odom_msg.header.seq = self.telegram_count
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation.w = rw
        odom_msg.pose.pose.orientation.x = rx
        odom_msg.pose.pose.orientation.y = ry
        odom_msg.pose.pose.orientation.z = rz
        pub.publish(odom_msg)

        # Increment telegram counter
        self.telegram_count = self.telegram_count + 1

    def startSendingTelegrams(self, pub):

        # Send a few telegram
        ts = 0
        prevts = 0
        # angVel = -10000
        # heading = 0
        # delta_heading = -10000 * 0.025 # 10 degree per second
        # while telegramCount < 300:
        #     if (telegramCount % 200) == 0:
        #         angVel = -angVel
        #         delta_heading = -delta_heading
        ts = int(time.time()*1000)
        linear_x = self.twistRbMa_odom.linear.x
        linear_y = 0
        angular_z = self.twistRbMa_odom.angular.z
        X_position = self.poseRbMa_odom.position.x
        Y_position = self.poseRbMa_odom.position.y

        orientation_w = self.poseRbMa_odom.orientation.w
        orientation_x = self.poseRbMa_odom.orientation.x
        orientation_y = self.poseRbMa_odom.orientation.y
        orientation_z = self.poseRbMa_odom.orientation.z

        self.sendOdometryTelegram(pub, linear_x, linear_y, angular_z, X_position, Y_position, orientation_w, orientation_x, orientation_y, orientation_z)
        # Delta between telegram timestamps
        print("Delta ts: ", ts  - prevts, " ms")
        # Update previous telegram timestamps
        prevts = ts
        # Sleep in order to reach 25ms odom telegram cycle time
        time.sleep(0.025 - (time.time() % 0.025))

    def shutdown(self):
        self.is_exit = 1
        
    ############################################ LOOP FUNCTION ########################################################################3
    def run(self):
        try:
            if self.is_exit == 0:
                while not rospy.is_shutdown():
                    if self.is_recv_odom == True:
                        self.startSendingTelegrams(self.pub_sick_odom)

                        # self.sick_odom_data.header = self.odom_data.header

                        # self.sick_odom_data.telegram_count += 1
                        # self.sick_odom_data.timestamp = int(rospy.get_time()*1000000) #us
                        # self.sick_odom_data.source_id = 31
                        # # print(self.poseRbMa_odom.position.x)
                        # self.sick_odom_data.x_position = round(self.poseRbMa_odom.position.x*1000)
                        # # print(self.sick_odom_data.x_position)
                        # # print(round(self.poseRbMa_odom.position.x*1000))

                        # self.sick_odom_data.y_position = round(self.poseRbMa_odom.position.y*1000)
                        # self.sick_odom_data.heading = round(degrees(self.theta_rb_odom)*1000)

                        # self.sick_odom_data.sync_timestamp_sec = 0
                        # self.sick_odom_data.sync_timestamp_nsec = 0
                        # self.sick_odom_data.sync_timestamp_valid = 0
                        
                        # self.pub_sick_odom.publish(self.sick_odom_data)

                    else:
                        rospy.logwarn("Chưa nhận được dữ liệu từ topic odom")

                    self.rate.sleep()
        except KeyboardInterrupt:
            rospy.on_shutdown(self.shutdown)
            self.is_exit = 1
            print('!!FINISH!!')

def main():
	print('Starting main program')
	program = ConvertMSG()
	program.run()
	print('Exiting main program')	

if __name__ == '__main__':
    main()
    