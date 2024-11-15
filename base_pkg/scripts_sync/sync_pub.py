#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Authors : BEE
# kinematic.
# DATE: 18/04/2021
# AUTHOR: HOANG VAN QUANG

import rospy
import sys
import time
import threading
import signal

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# from sti_msgs.msg import Driver_query
# from sti_msgs.msg import Driver_respond
from geometry_msgs.msg import PointStamped

from math import sin , cos , pi , atan2

class sync_pub():
	def __init__(self):
		# ---------------------------------------- ROS
		print("ROS Initial!")
		rospy.init_node('sync_pub', anonymous=False) # False
		self.rate = rospy.Rate(1000)
		self.lastTime_pub1 = time.time()
		self.time_pub1 = 1/4. # s
		self.lastTime_pub2 = time.time()
		self.time_pub2 = 1. # s		

		self.pub_pointStamped_1 = rospy.Publisher("pointStamped1", PointStamped, queue_size=50)
		self.pointStamped_1 = PointStamped()

		self.pub_pointStamped_2 = rospy.Publisher("pointStamped2", PointStamped, queue_size=50)
		self.pointStamped_2 = PointStamped()

	def run(self):
		print "Launch ALL!"
		while not rospy.is_shutdown(): # not self.shutdown_flag.is_set() or 
			t1 = (time.time() - self.lastTime_pub1)%60
			if (t1 > self.time_pub1):
				self.lastTime_pub1 = time.time()
				self.pointStamped_1.header.stamp = rospy.Time.now()
				self.pub_pointStamped_1.publish(self.pointStamped_1)

			t2 = (time.time() - self.lastTime_pub2)%60
			if (t2 > self.time_pub2):
				self.lastTime_pub2 = time.time()
				self.pointStamped_2.header.stamp = rospy.Time.now()
				self.pub_pointStamped_2.publish(self.pointStamped_2)

			self.rate.sleep()

		print('program stopped')


def main():
	print('Starting main program')
	programer = sync_pub()
	programer.run()

	print('Exiting main program')
	sys.exit()

if __name__ == '__main__':
    main()	

# 1617248609 549340009
# 1617248609 590301990