#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#Authors : BEE
# kinematic.
# DATE: 21/06/2021
# AUTHOR: HOANG VAN QUANG

import rospy
import sys
import time
import threading
import signal

from geometry_msgs.msg import Twist
from message_pkg.msg import Driver_respond

from sti_msgs.msg import Velocities

from math import sin , cos , pi , atan2

class filterTopic():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('filter_topic', anonymous=False) # False
		

		# -- parameter
		self.pub_frequence = rospy.get_param("frequence", 25.)
		self.topicSub_driver1 = rospy.get_param("topicSub_driver1", "/driver1_respond")
		self.topicSub_driver2 = rospy.get_param("topicSub_driver2", "/driver2_respond")
		self.topicPub_driver1 = rospy.get_param("topicPub_driver1", "/driver1_filter")
		self.topicPub_driver2 = rospy.get_param("topicPub_driver2", "/driver2_filter")

		self.rate = rospy.Rate(self.pub_frequence)
		# -- SUB
		rospy.Subscriber(self.topicSub_driver1, Driver_respond, self.driver1Respond_callback)
		self.driver1_respond = Driver_respond()

		rospy.Subscriber(self.topicSub_driver2, Driver_respond, self.driver2Respond_callback)
		self.driver2_respond = Driver_respond()

		# -- PUB
		self.pub_filter1 = rospy.Publisher(self.topicPub_driver1, Driver_respond, queue_size=50)
		self.driver1_filter = Driver_respond()

		self.pub_filter2 = rospy.Publisher(self.topicPub_driver2, Driver_respond, queue_size=50)
		self.driver2_filter = Driver_respond()
		# --
		self.time_pub = 1/self.pub_frequence # s
		self.preTime_pub1 = time.time()
		self.preTime_pub2 = time.time()
		# -- 
		self.nowTime_speed1 = time.time()
		self.nowTime_speed2 = time.time()
		self.is_timeout1 = 0
		self.is_timeout2 = 0
		self.timeout = self.time_pub*3.

	def driver1Respond_callback(self, data):
		self.driver1_respond = data
		self.nowTime_speed1 = time.time()

	def driver2Respond_callback(self, data):
		self.driver2_respond = data
		self.nowTime_speed2 = time.time()

	def run(self):
		print ("Launch ALL!")
		while not rospy.is_shutdown(): # not self.shutdown_flag.is_set() or 
			# --
			t1_out = time.time() - self.nowTime_speed1
			if (t1_out > self.timeout):
				self.is_timeout1 = 1
			else:
				self.is_timeout1 = 0
			# --
			t2_out = time.time() - self.nowTime_speed2
			if (t2_out > self.timeout):
				self.is_timeout2 = 1
			else:
				self.is_timeout2 = 0
			# --
			if (self.is_timeout1 == 0):
				self.pub_filter1.publish(self.driver1_respond)
			# --
			if (self.is_timeout2 == 0):
				self.pub_filter2.publish(self.driver2_respond)

			self.rate.sleep()
		print('program stopped')

def main():
	print('Starting main program')

	program = filterTopic()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()
