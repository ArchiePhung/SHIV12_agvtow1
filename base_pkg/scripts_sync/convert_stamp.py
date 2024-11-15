#!/usr/bin/env python
# -*- coding: utf-8 -*-

# DATE: 01/04/2021
# AUTHOR: HOANG VAN QUANG

import rospy
import sys
import time
import threading
import signal

from geometry_msgs.msg import Twist
from sti_msgs.msg import Stampe_data
from math import sin , cos , pi , atan2

class convertStamp():
	def __init__(self):
		# ---------------------------------------- ROS
		print("ROS Initial!")
		rospy.init_node('convert_stamp', anonymous=False) # False
		self.rate = rospy.Rate(1)

		rospy.Subscriber("raw_vel", Stampe_data, self.rawVel_infoCallback)

		rospy.Subscriber("raw_imu", Stampe_data, self.rawImu_infoCallback)

		self.pub_stampe_vel = rospy.Publisher("stampe_vel", Stampe_data, queue_size = 4)
		self.stampe_vel = Stampe_data()

		self.pub_stampe_imu = rospy.Publisher("stampe_imu", Stampe_data, queue_size = 4)
		self.stampe_imu = Stampe_data()

		self.pre_time = rospy.Time.now() 

	def rawVel_infoCallback(self, data):
		self.stampe_vel.header.stamp = rospy.Time.now() 
		self.stampe_vel.lx = data.linear_x
		self.stampe_vel.gx = data.angular_z
		self.pub_stampe_vel.publish(self.stampe_vel)

	def rawImu_infoCallback(self, data):
		self.stampe_imu.header.stamp = rospy.Time.now() 
		self.stampe_imu.gx = data.angular_velocity.z
		self.pub_stampe_imu.publish(self.stampe_imu)

	def run(self):
		print "Launch ALL!"
		while not rospy.is_shutdown():
			a = rospy.Time.now() - self.pre_time
			print ("time now: ", a)
			self.pre_time = rospy.Time.now()
			self.rate.sleep()

		print('program stopped')


def main():
	print('Starting main program')
	programer = convertStamp()
	programer.run()

	print('Exiting main program')
	sys.exit()

if __name__ == '__main__':
    main()	
