#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Authors : BEE
# DATE: 01/07/2021
# AUTHOR: HOANG VAN QUANG - BEE

import rospy
import sys
import time
import roslaunch
import os
import subprocess

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan , Image, Imu

from sti_msgs.msg import *
from message_pkg.msg import *
"""
yêu cầu thông tin để kết nối lại 1 node:
1, Topic để kiểm tra node đó có đang hoạt động không.
2, Tên node để shutdown node đó.
3, File launch khởi tạo node.
----------------
Nếu cổng vật lý vẫn còn.
"""

class Reconnect:
	def __init__(self, nameTopic_sub, time_checkLost, time_waitLaunch, file_launch):
		# -- parameter
		self.time_checkLost = time_checkLost
		self.time_waitLaunch = time_waitLaunch # wait after launch
		self.fileLaunch = file_launch
		self.nameTopic_sub = nameTopic_sub
		self.nameNode = ''
		self.is_nameReaded = 0
        # -- launch
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)
		# -- variable
		self.lastTime_waitConnect = time.time()
		self.time_readed = time.time()
		self.enable_check = 0
		self.process = 0
		self.numberReconnect = 0
		# -- 
		self.lastTime_waitShutdown = time.time()
		
	def read_nameNode(self, topic):
		try:
			output = subprocess.check_output("rostopic info {}".format(topic), shell= True)
			# print("out: ", output)

			pos1 = str(output).find('Publishers:') # tuyet doi ko sua linh tinh.
			pos2 = str(output).find(' (http://')   # tuyet doi ko sua linh tinh.
			# print("pos2: ", pos2)
			if (pos1 >= 0):
				name = str(output)[pos1 + 16 :pos2]
				p1 = name.find('Subscribers:')
				p2 = name.find('*')
				if (p1 == -1 and p2 == -1):
					return name
				else:
					print ("p1: ", p1)
					print ("p2: ", p2)
					print ("Read name error: " + name + "|" + self.nameTopic_sub)
					return ''
			else:
				# print ("Not!")
				return ''

		except Exception as e:
			# print ("Error!")
			return ''

	def run_reconnect_vs2(self, enb_check, time_readed): # have kill node via name_topic pub.
		self.enable_check = enb_check
		self.time_readed = time_readed
		# -- read name node.
		if (enb_check == 1 and self.is_nameReaded == 0):
			self.nameNode = "driverELD2_node" # self.read_nameNode(self.nameTopic_sub)

			if (len(self.nameNode) != 0):
				self.is_nameReaded = 1
		# -- 	
		launch = roslaunch.parent.ROSLaunchParent(self.uuid , [self.fileLaunch])
		if (self.process == 0): # - Check lost.
			if (self.enable_check):
				t = time.time() - self.time_readed
				if (t >= self.time_checkLost):
					print("Detected Lost: " + str(self.nameNode))
					self.process = 1

		elif (self.process == 1): # - shutdown node
			launch.shutdown()
			print ("shutdown node: ", str(self.nameNode))

			if (self.is_nameReaded):
				# self.nameNode = "/lineMagnetic"
				try:
					os.system("rosnode kill " + self.nameNode)
					print ("rosnode kill " + self.nameNode)
				except:
					print ("ERROR rosnode kill /" + self.nameNode)

			self.lastTime_waitShutdown = time.time()
			print ("Wait after shutdown node: " + self.nameNode)
			self.process = 2

		elif (self.process == 2): # - wait after shutdown.
			t = time.time() - self.lastTime_waitShutdown
			if (t >= 3):
				self.process = 3

		elif (self.process == 3): # - Launch
			print ("Launch node: " + self.nameNode)
			launch.start()
			self.numberReconnect += 1
			self.lastTime_waitConnect = time.time()
			self.process = 4

		elif (self.process == 4): # - wait after launch
			t1 = time.time() - self.lastTime_waitConnect
			t2 = time.time() - self.time_readed # have topic pub
			if (t1 >= self.time_waitLaunch or t2 <= 1):
				# print ("t1: ", t1)
				# print ("t2: ", t2)
				print ("Launch node completed: " + self.nameNode)
				self.process = 0
				self.is_nameReaded = 0
				self.nameNode = ''

		return self.process, self.numberReconnect

	def run_reconnect_vs3(self, enb_check, time_readed, off_kill): # have kill node via name_topic pub. + Add funtion turn of rosnode kill
		self.enable_check = enb_check
		self.time_readed = time_readed
		# -- read name node.
		if (enb_check == 1 and self.is_nameReaded == 0):
			self.nameNode = self.read_nameNode(self.nameTopic_sub)
			if (len(self.nameNode) != 0):
				self.is_nameReaded = 1
		# -- 	
		launch = roslaunch.parent.ROSLaunchParent(self.uuid , [self.fileLaunch])
		if (self.process == 0): # - Check lost.
			if (self.enable_check):
				t = time.time() - self.time_readed
				if (t >= self.time_checkLost):
					print ("Detected Lost: " + str(self.nameNode))
					self.process = 1

		elif (self.process == 1): # - shutdown node
			launch.shutdown()
			print ("shutdown node: ", str(self.nameNode))

			if (off_kill == 0):
				if (self.is_nameReaded):
					# self.nameNode = "/lineMagnetic"
					os.system("rosnode kill " + self.nameNode)
					print ("rosnode kill " + self.nameNode)

			self.lastTime_waitShutdown = time.time()
			print ("Wait after shutdown node: " + self.nameNode)
			self.process = 2

		elif (self.process == 2): # - wait after shutdown.
			t = time.time() - self.lastTime_waitShutdown
			if (t >= 3):
				self.process = 3

		elif (self.process == 3): # - Launch
			print ("Launch node: " + self.nameNode)
			launch.start()
			self.numberReconnect += 1
			self.lastTime_waitConnect = time.time()
			self.process = 4

		elif (self.process == 4): # - wait after launch
			t1 = time.time() - self.lastTime_waitConnect
			t2 = time.time() - self.time_readed # have topic pub
			if (t1 >= self.time_waitLaunch or t2 <= 1):
				# print ("t1: ", t1)
				# print ("t2: ", t2)
				print ("Launch node completed: " + self.nameNode)
				self.process = 0
				self.is_nameReaded = 0
				self.nameNode = ''

		return self.process, self.numberReconnect

	def run_reconnect(self, enb_check, time_readed): 
		self.enable_check = enb_check
		self.time_readed = time_readed

		launch = roslaunch.parent.ROSLaunchParent(self.uuid , [self.fileLaunch])
		if (self.process == 0): # - Check lost.
			if (self.enable_check):
				t = time.time() - self.time_readed
				if (t >= self.time_checkLost):
					print ("Detected Lost: " + str(self.nameNode))
					self.process = 1

		elif (self.process == 1): # - shutdown node
			launch.shutdown()
			print ("shutdown node: ", str(self.nameNode))
			self.lastTime_waitShutdown = time.time()

			print ("Wait after shutdown node: " + self.nameNode)
			self.process = 2

		elif (self.process == 2): # - wait after shutdown.
			t = time.time() - self.lastTime_waitShutdown
			if (t >= 3):
				self.process = 3

		elif (self.process == 3): # - Launch
			print ("Launch node: " + self.nameNode)
			launch.start()
			self.numberReconnect += 1
			self.lastTime_waitConnect = time.time()
			self.process = 4

		elif (self.process == 4): # - wait after launch
			t1 = time.time() - self.lastTime_waitConnect
			t2 = time.time() - self.time_readed # have topic pub
			if (t1 >= self.time_waitLaunch or t2 <= 1):
				# print ("t1: ", t1)
				# print ("t2: ", t2)
				print ("Launch node completed: " + self.nameNode)
				self.process = 0
				
		return self.process, self.numberReconnect

class Reconnect_node():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('reconnectDrivers_node', anonymous=False)
		self.rate = rospy.Rate(100)
		# -- Port
		# rospy.Subscriber("/status_port", Status_port, self.callback_port)
		# self.port_status = Status_port()
		# -- PUB
		self.pub_statusReconnect = rospy.Publisher('/status_reconnectDriverAll', Status_reconnect, queue_size= 10)
		self.statusReconnect = Status_reconnect()
		self.pre_timePub = time.time()
		self.cycle_timePub = 0.4 # s

		# -- -- Driverall ------------------------------------------------
		self.path_driverAll = rospy.get_param("path_driverAll", "/home/stivietnam/catkin_ws/src/base_pkg/launch/drivers.launch")
		# -- -- -- driverAll
		self.timeLost_driverAll = 2 # rospy.get_param("timeLost_driver", 2)
		self.timeWait_driverAll = 5 # rospy.get_param("timeWait_driver", 5)
		self.topicSub_driverAll = "/driver2_respond"
		self.isRuned_driverAll  = 0
		self.timeReaded_driverAll = time.time()
		rospy.Subscriber(self.topicSub_driverAll, Driver_respond, self.callback_driverAll)
		self.reconnect_driverAll = Reconnect(self.topicSub_driverAll, self.timeLost_driverAll, self.timeWait_driverAll, self.path_driverAll)
		# -- add new 23/11/2022
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.callback_stiControl)

	# def callback_port(self, data):
	# 	self.port_status = data

	def callback_stiControl(self, data):
		self.isRuned_driverAll = 1

	def callback_driverAll(self, data):
		self.isRuned_driverAll = 1
		self.timeReaded_driverAll = time.time()

	def reconnect_node(self):
		# -- driverAll
		process, num = self.reconnect_driverAll.run_reconnect_vs2(self.isRuned_driverAll, self.timeReaded_driverAll)
		self.statusReconnect.driverAll.sts = process
		self.statusReconnect.driverAll.times = num

		# -- -- -- 
		tim = (time.time() - self.pre_timePub)%60
		if (tim > self.cycle_timePub):
			self.pre_timePub = time.time()
			self.pub_statusReconnect.publish(self.statusReconnect)

	def run(self):
		while not rospy.is_shutdown():
			self.reconnect_node()

			self.rate.sleep()

		print('Programer stopped')

def main():
	print('Starting main program - Reconnect Driver')

	program = Reconnect_node()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()
