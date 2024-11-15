#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import time
import signal

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped, Quaternion

from sti_msgs.msg import EncoderData
from std_msgs.msg import Int8

from math import sin , cos , pi , atan2, degrees, radians, fabs

from tf.transformations import quaternion_from_euler

from sick_lidar_localization.msg import EncoderMeasurementMessage0202

from sick_lidar_localization.srv import LocSetOdometryActiveSrv


class OdomEncoder():
  def __init__(self):
    print("ROS Initial!")
    rospy.init_node('encoder_lidarLOC', anonymous= False) # False
    self.rate = rospy.Rate(30)

    # -----------------
    rospy.Subscriber("/encoder_respond", EncoderData, self.encoder_callback)

    # -----------------
    # rospy.Subscriber("/raw_vel", TwistWithCovarianceStamped, self.rawVel_callback)
    self.is_stop = -1

    self.data_ralVel = 0

    # -----------------
    self.pub_EncoderData = rospy.Publisher("/localizationcontroller/in/encoder_measurement_message_0202", EncoderMeasurementMessage0202, queue_size=10)

    self.ID_SENDER_ELEFT = 21
    self.ID_SENDER_ERIGHT = 22
    
    # -- 
    self.tick_encoderLeft = None
    self.tick_encoderRight = None

    self.delta_enLeft = 0
    self.delta_enright = 0

    self.telegram_count = 0
    self.ts = 0
    self.prevts = 0
    self.toggle_encoder = 0

    self.start = 0

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
    self.ts = int(time.time()*1000000)
    # - send left encoder data
    if self.toggle_encoder == 0:
      source_id = self.ID_SENDER_ELEFT
      encoder_val = self.delta_enLeft

    else:
      source_id = self.ID_SENDER_ERIGHT
      encoder_val = self.delta_enright

    self.sendEncoderTelegram(pub, self.ts, source_id, encoder_val)
    # print("Delta ts: ", self.ts  - self.prevts, " microsec")
    self.prevts = self.ts
    time.sleep(0.025 - (time.time() % 0.025))

  def encoder_callback(self, data):
    self.start = 1
    print(data.encoder_left_val, data.encoder_right_val)
    self.delta_enLeft = data.encoder_left_val
    self.delta_enright = data.encoder_right_val

  def run(self):
    while not rospy.is_shutdown():
      if self.start == 1:
        self.startSendingTelegrams(self.pub_EncoderData)

        # if fabs(self.delta_enLeft) > 10 and fabs(self.delta_enright) > 10:
        #   self.data_ralVel = 0
        # else:
        #   self.data_ralVel = 1

        # if self.data_ralVel != self.is_stop:
        #   self.is_stop = self.data_ralVel
        #   print("----------------", self.is_stop)
        #   try:
        #     serClient = rospy.ServiceProxy('/LocSetOdometryActive', LocSetOdometryActiveSrv)
        #     resp = serClient(self.is_stop)
        #     print(resp.success)

        #   except rospy.ServiceException as e:
        #     rospy.logwarn(e)

      self.rate.sleep()

def main():
  print('Starting main program')

  program = OdomEncoder()
  program.run()
  print('Exiting main program')	

if __name__ == '__main__':
    main()	






