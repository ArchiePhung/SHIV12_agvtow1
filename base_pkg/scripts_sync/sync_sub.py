#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Authors : BEE
# kinematic.
# DATE: 18/03/2021
# AUTHOR: HOANG VAN QUANG

import rospy
import sys
import time
import threading
import signal

from nav_msgs.msg import Odometry
from sti_msgs.msg import Stampe_data

from math import sin , cos , pi , atan2

class sync_sub():
	def __init__(self):
		# ---------------------------------------- ROS
		print("ROS Initial!")
		rospy.init_node('sync_sub', anonymous=False) # False
		self.rate = rospy.Rate(1000)
		#   0   |   1  |  2  |  3 
		#  pre2 | pre1 | now | fur
		rospy.Subscriber("stampe_vel", Stampe_data, self.vel_infoCallback)
		self.vel_fur = Stampe_data()
		self.vel_now = Stampe_data()
		self.vel_pre1 = Stampe_data()
		self.vel_pre2 = Stampe_data()
		self.is_newVel = 0
		self.is_readVel = 0

		rospy.Subscriber("stampe_imu", Stampe_data, self.imu_infoCallback)
		self.imu_fur = Stampe_data()
		self.imu_now = Stampe_data()
		self.imu_pre1 = Stampe_data()
		self.imu_pre2 = Stampe_data()
		self.is_newImu = 0
		self.is_readImu = 0

		self.pub_stampe_imu = rospy.Publisher("stampe_finnal", Stampe_data, queue_size = 4)
		self.stampe_finnal = Stampe_data()

		self.process = 0
		# check time out
		self.time_vel = 0.5 # s
		self.time_imu = 0.5 # s
		self.lastTime_vel = time.time()
		self.lastTime_imu = time.time()

	def vel_infoCallback(self, data):
		self.vel_pre2 = self.vel_pre1
		self.vel_pre1 = self.vel_now
		self.vel_now = data
		self.is_newVel = 1
		self.is_readVel = 1

	def imu_infoCallback(self, data):
		self.imu_pre2 = self.imu_pre1
		self.imu_pre1 = self.imu_now
		self.imu_now = data
		self.is_newImu = 1
		self.is_readImu = 1

	def check_timeOut_vel(self):
		if (self.is_newVel == 1):
			self.lastTime_vel = time.time()
		else:
			t_vel = time.time() - self.lastTime_vel
			if (t_vel > self.time_vel):
				return 1
		return 0

	def check_timeOut_imu(self):
		if (self.is_newImu == 1):
			self.lastTime_ium = time.time()
		else:
			t_imu = time.time() - self.lastTime_imu
			if (t_imu > self.time_imu):
				return 1
		return 0

	def run(self):
		print "Launch ALL!"

		while not rospy.is_shutdown():
			if (self.process == 0):
				ct = 0
				if (self.is_readVel == 1)
					ct += 1
				else:
					print ("Wait read Vel!")

				if (self.is_readImu == 1)
					ct += 1
				else:
					print ("Wait read Imu!")
				
				if (ct == 2):
					self.process = 1
			else:
				if (self.is_newVel == 1)
					self.is_newVel = 0
					t_lech = (self.vel_now.stamp - self.vel_now.stamp)*
					if (t_lech < )
					# alpha_val = self.imu_pre1.gx - self.imu_pre2.gx
					if (alpha_val != 0):
						alpha_time = self.imu_pre1.stamp - self.imu_pre2.stamp # nano second
						val_imu = alpha_val/alpha_time
				
			self.rate.sleep()

		print('program stopped')


def main():
	print('Starting main program')
	programer = sync_sub()
	programer.run()

	print('Exiting main program')
	sys.exit()

if __name__ == '__main__':
    main()	
