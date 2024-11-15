#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dev: Archie Phung
Date Modify: 16/8/2024

Advantage:
  > Convert encoder data for lidar loc 2 can use

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
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose, PoseStamped, Twist, PoseWithCovarianceStamped

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from message_pkg.msg import *
from sti_msgs.msg import *
from ros_canBus.msg import *
from sick_lidar_localization.msg import EncoderMeasurementMessage0202, LocalizationControllerResultMessage0502

import os
import time

class ConvertMSG():
    def __init__(self):
        print("ROS Initial: Convert_odom_to_Sickodom105 !")
        rospy.init_node('convert_odom_to_sickodom105_right', anonymous = False, disable_signals=True) # False

        self.rate = rospy.Rate(40)

        # self.tolerance_rot_step1 = rospy.get_param('~tolerance_rot_step1',0.02)
        # self.vel_rot_step1 = rospy.get_param('~vel_rot_step1',0.32)      # 0.45
        self.vel_rot_step1 = 0.2

        # -- SUBSCRIBER NODE -- 

        # -- Keyboard command
        rospy.Subscriber("/encoder_respond", EncoderData, self.callback_Encoder) # or odom_combined
        # self.poseRbMa_odom = Pose()
        self.Encoder_data = EncoderData()
        self.is_recv_encoder = False

        # -- PUBLISH TOPIC -- 
        self.pub_EncoderData = rospy.Publisher("/localizationcontroller/in/encoder_measurement_message_0202", EncoderMeasurementMessage0202, queue_size=10)
        self.lls_encoder_data = EncoderMeasurementMessage0202()
        
        # -- PUBLISH TOPIC -- 
        self.pub_EncoderRequest = rospy.Publisher("/encoder_request", Int8, queue_size=10)
        self.data_EncoderRequest = Int8()

        # -- CONST --
        self.rate_sickOdom = 33
        self.ID_SENDER_ELEFT = 21
        self.ID_SENDER_ERIGHT = 22

        self.time_tr = rospy.get_time()
        self.is_exit = 0
        self.theta_rb_odom = 0.0

        self.telegram_count = 0
        self.ts = 0
        self.prevts = 0
        self.toggle_encoder = 0
        self.step = 0

    ############################################ CALLBACK FUNCTION ########################################################################3
    def callback_Encoder(self, data):
        self.Encoder_data = data
        self.is_recv_encoder = True

    ############################################ DEF FUNCTION ########################################################################3
    def sendEncoderTelegram(self, pub, ts, id, val):
        encoder_msg = EncoderMeasurementMessage0202()
        encoder_msg.telegram_count = self.telegram_count
        encoder_msg.timestamp = ts
        encoder_msg.source_id = id
        encoder_msg.encoder_value = val
        pub.publish(encoder_msg)

        # Increment telegram counter
        self.telegram_count = self.telegram_count + 1
        self.toggle_encoder = not self.toggle_encoder

    def startSendingTelegrams(self, pub):
        # Send a few telegram

        self.ts = int(time.time()*1000000)

        # - send left encoder data
        # if self.toggle_encoder == 0:
        #     source_id = self.ID_SENDER_ELEFT
        #     encoder_val = self.Encoder_data.encoder_left_val

        # else:
        #     source_id = self.ID_SENDER_ERIGHT
        #     encoder_val = self.Encoder_data.encoder_right_val

        self.sendEncoderTelegram(pub, self.ts, self.ID_SENDER_ERIGHT, self.Encoder_data.encoder_right_val)

        # heading = heading + delta_heading
        # Delta between telegram timestamps
        print("Delta ts: ", self.ts  - self.prevts, " microsec")
        # Update previous telegram timestamps
        self.prevts = self.ts
        # Sleep in order to reach 25ms odom telegram cycle time
        time.sleep(0.025 - (time.time() % 0.025))

    def shutdown(self):
        self.is_exit = 1
        
    ############################################ LOOP FUNCTION ########################################################################3
    def run(self):
        try:
            if self.is_exit == 0:
                while not rospy.is_shutdown():
                    if self.step == 0:
                        print("reset encoder val")
                        self.data_EncoderRequest.data = 1
                        if self.is_recv_encoder == True and self.Encoder_data.encoder_left_val == 0:
                            self.data_EncoderRequest.data = 0
                            self.step = 1
                    else:

                        self.startSendingTelegrams(self.pub_EncoderData)

                    self.pub_EncoderRequest.publish(self.data_EncoderRequest)

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
    