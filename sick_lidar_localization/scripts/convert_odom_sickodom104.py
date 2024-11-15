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
from sick_lidar_localization.msg import OdometryMessage0104, LocalizationControllerResultMessage0502

import os
import time

class ConvertMSG():
    def __init__(self):
        print("ROS Initial: Convert_odom_to_Sickodom104 !")
        rospy.init_node('convert_odom_to_sickodom104', anonymous = False, disable_signals=True) # False

        self.rate = rospy.Rate(40)

        # self.tolerance_rot_step1 = rospy.get_param('~tolerance_rot_step1',0.02)
        # self.vel_rot_step1 = rospy.get_param('~vel_rot_step1',0.32)      # 0.45
        self.vel_rot_step1 = 0.2

        # -- SUBSCRIBER NODE -- 
        # -- Keyboard command
        rospy.Subscriber("/raw_vel", TwistWithCovarianceStamped, self.callback_Rawvel) # or odom_combined
        self.data_vel = Twist()
        self.is_recv_vel = False

#         rospy.Subscriber("/localizationcontroller/out/localizationcontroller_result_message_0502", LocalizationControllerResultMessage0502
# , self.callback_lls) # 
#         self.lls_data = LocalizationControllerResultMessage0502()
#         self.is_recv_lls = False

        # -- PUBLISH TOPIC -- 
        self.pub_sick_vel = rospy.Publisher("/localizationcontroller/in/odometry_message_0104", OdometryMessage0104, queue_size=10)
        self.sick_data_vel = OdometryMessage0104()
        
        # -- CONST --
        self.rate_sickOdom = 40
        self.ID_SENDER = 41

        self.time_tr = rospy.get_time()
        self.is_exit = 0
        self.theta_rb_odom = 0.0

        self.telegram_count = 0

    ############################################ CALLBACK FUNCTION ########################################################################3
    def callback_Rawvel(self, data):
        self.data_vel = data.twist.twist
        self.is_recv_vel = True

    ############################################ DEF FUNCTION ########################################################################3
    def sendOdometryTelegram(self, pub, id, vx, vy, omega, ts):
        vel_msg = OdometryMessage0104()
        vel_msg.telegram_count = self.telegram_count
        vel_msg.timestamp = ts
        vel_msg.source_id = id
        vel_msg.x_velocity = vx
        vel_msg.y_velocity = vy
        vel_msg.angular_velocity = omega
        pub.publish(vel_msg)

        # Increment telegram counter
        self.telegram_count = self.telegram_count + 1

    def startSendingTelegrams(self, pub):
        # Send a few telegram
        ts = 0
        prevts = 0
        source_id = self.ID_SENDER
        # heading = 0
        # delta_heading = 10000 * 0.025 # 10 degree per second
        # while telegramCount < 300:
            # if (telegramCount % 200) == 0:
                # delta_heading = -delta_heading

        ts = int(time.time()*1000000)
        X_vel = round(self.data_vel.linear.x*1000)
        Y_vel = 0
        angular_vel = round(self.data_vel.angular.z*180000/ PI)

        self.sendOdometryTelegram(pub, source_id, X_vel, Y_vel, angular_vel, ts)

        # heading = heading + delta_heading
        # Delta between telegram timestamps
        print("Delta ts: ", ts  - prevts, " microsec")
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
                    if self.is_recv_vel == True:
                        self.startSendingTelegrams(self.pub_sick_vel)

                    else:
                        rospy.logwarn("Chưa nhận được dữ liệu từ topic vel")

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
    