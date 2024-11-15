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

        self.rate = rospy.Rate(40)

        # self.tolerance_rot_step1 = rospy.get_param('~tolerance_rot_step1',0.02)
        # self.vel_rot_step1 = rospy.get_param('~vel_rot_step1',0.32)      # 0.45
        self.vel_rot_step1 = 0.2

        # -- SUBSCRIBER NODE -- 
        # -- Keyboard command
        rospy.Subscriber("/odom_lms100", Odometry, self.callback_Odometry) # or odom_combined
        self.poseRbMa_odom = Pose()
        self.odom_data = Odometry()
        self.is_recv_odom = False

#         rospy.Subscriber("/localizationcontroller/out/localizationcontroller_result_message_0502", LocalizationControllerResultMessage0502
# , self.callback_lls) # 
#         self.lls_data = LocalizationControllerResultMessage0502()
#         self.is_recv_lls = False

        # -- PUBLISH TOPIC -- 
        self.pub_sick_odom = rospy.Publisher("/localizationcontroller/in/odometry_message_0105", OdometryMessage0105, queue_size=10)
        self.sick_odom_data = OdometryMessage0105()
        
        # -- CONST --
        self.rate_sickOdom = 33
        self.ID_SENDER = 31

        self.time_tr = rospy.get_time()
        self.is_exit = 0
        self.theta_rb_odom = 0.0

        self.telegram_count = 0
        self.ts = 0
        self.prevts = 0

    ############################################ CALLBACK FUNCTION ########################################################################3
    def callback_Odometry(self, data):
        self.odom_data = data
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

    # def callback_lls(self, data):
    #     self.is_recv_lls = True
    #     self.lls_data = data

    ############################################ DEF FUNCTION ########################################################################3
    def sendOdometryTelegram(self, pub, id, x, y, heading, ts):
        odom_msg = OdometryMessage0105()
        odom_msg.telegram_count = self.telegram_count
        odom_msg.timestamp = self.ts
        odom_msg.source_id = id
        odom_msg.x_position = x
        odom_msg.y_position = y
        odom_msg.heading = heading
        pub.publish(odom_msg)

        # Increment telegram counter
        self.telegram_count = self.telegram_count + 1

    def startSendingTelegrams(self, pub):
        # Send a few telegram
        # ts = 0
        # prevts = 0
        source_id = self.ID_SENDER
        # heading = 0
        # delta_heading = 10000 * 0.025 # 10 degree per second
        # while telegramCount < 300:
            # if (telegramCount % 200) == 0:
                # delta_heading = -delta_heading

        self.ts = int(time.time()*1000000)
        X_position = round(self.poseRbMa_odom.position.x*1000)
        Y_position = round(self.poseRbMa_odom.position.y*1000)
        heading = round(degrees(self.theta_rb_odom)*1000)

        if heading < -180000:
            heading = -180000
        elif heading > 180000:
            heading = 180000

        self.sendOdometryTelegram(pub, source_id, X_position, Y_position, int(heading), self.ts)

        # heading = heading + delta_heading
        # Delta between telegram timestamps
        print("Delta ts: ", self.ts  - self.prevts, " microsec")
        # Update previous telegram timestamps
        self.prevts = self.ts
        # Sleep in order to reach 25ms odom telegram cycle time
        time.sleep(0.03 - (time.time() % 0.03))

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
    