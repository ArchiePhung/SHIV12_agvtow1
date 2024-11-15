#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Authors : BEE
# DATE: 09/06/2021
# AUTHOR: HOANG VAN QUANG - BEE

import rospy
import sys
import time
import roslaunch
import os

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

from sti_msgs.msg import *

"""
yêu cầu thông tin để kết nối lại 1 node:
1, Topic để kiểm tra node đó có đang hoạt động không.
2, Tên node để shutdown node đó.
3, File launch khởi tạo node.

"""
class Reconnect:
	def __init__(self, name_node, time_checkLost, time_waitLaunch, file_launch):
		# -- parameter
		self.time_checkLost = time_checkLost
		self.time_waitLaunch = time_waitLaunch # wait after launch
		self.file_launch = file_launch
		self.name_node = name_node
        # -- launch
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
		# -- 
		self.lastTime_waitConnect = time.time()
		self.time_readed = time.time()
		self.enable_check = 0
		self.process = 0
		# -- 
		self.lastTime_waitShutdown = time.time()

	def run_reconnect(self, enb_check, time_readed):
		self.enable_check = enb_check
		self.time_readed = time_readed

		if (self.process == 0): # - Check lost.
			if (self.enb_check):
				t = time.time() - self.time_readed
				if (t >= self.time_checkLost):
					print ("Detected Lost: " + str(self.name_node))
					self.process = 1

		elif (self.process == 1): # - shutdown node
			launch = roslaunch.parent.ROSLaunchParent(self.uuid , [self.file_launch])
			launch.shutdown()
			print ("shutdown node: ", self.name_node)
			os.system("rosnode kill " + self.name_node)
			self.lastTime_waitShutdown = time.time()

			print ("Wait after shutdown node: " + self.name_node)
			self.process = 2

		elif (self.process == 2): # - wait after shutdown.
			t = time.time() - self.lastTime_waitShutdown
			if (t >= 3):
				self.process = 3

		elif (self.process == 3): # - Launch
			print ("Launch node: " + self.name_node)
			launch.start()
			self.lastTime_waitConnect = time.time()
			self.process = 4

		elif (self.process == 4): # - wait after launch
			t1 = time.time() - self.lastTime_waitConnect
			t2 = time.time() - self.time_readed # have topic pub
			if (t1 >= self.time_waitLaunch or t2 <= 1):
				print ("Launch node completed: " + self.name_node)
				self.process = 0

class reconnect_driver():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('driver_reconnect', anonymous=False)
		self.rate = rospy.Rate(100)
		# -- paremeter
		self.timeCheck_connect = rospy.get_param("timeCheck_connect", 5)
		self.timeWait_connect = rospy.get_param("timeWait_connect", 6)
		self.topicCheck = rospy.get_param("topicCheck", '')
		self.name_node = rospy.get_param("name_node", '')
		self.file_launch = rospy.get_param("file_launch", '')

		rospy.Subscriber(self.topicCheck, Int16, self.runing_infoCallback)
		self.driver_runing = Int16()
		self.is_runing = 0
		self.time_readed = time.time()

		# -- define reconnect object.
		self.driver_reconnect = Reconnect(self.name_node, self.timeCheck_connect, self.timeWait_connect, self.file_launch)

	def runing_infoCallback(self, data):
		self.driver_runing = data
		self.is_runing = 1
		self.time_readed = time.time()

	def reconnect_node(self):
		self.driver_reconnect.run_reconnect(self.is_runing, self.time_readed)

	def run(self):
		while not rospy.is_shutdown():
			self.reconnect_node()

			self.rate.sleep()

		print('Programer stopped')

def main():
	print('Starting main program')

	program = driver()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()
