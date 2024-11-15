#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Authors : BEE
# blv 200W.
# DATE: 23/03/2021
# AUTHOR: HOANG VAN QUANG

import rospy
import sys
import time
import threading
import signal

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sti_msgs.msg import Driver_query
from std_msgs.msg import Bool
from sti_msgs.msg import Driver_respond

from math import sin , cos , pi , atan2

class testRun():
	def __init__(self):
		# ---------------------------------------- ROS
		print("ROS Initial!")
		rospy.init_node('testRun', anonymous=False)
		self.rate = rospy.Rate(100)

		self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=50)
		self.cmd_vel = Twist()

		self.process = 0
		self.t_rotation = 5.5
		self.t_move = 3
		self.vel_rotation = 0.3
		self.vel_move = 0.4
		self.lastTime = time.time()

	def run(self):
		while not rospy.is_shutdown():

			if self.process == 0: # move
				t = (time.time() - self.lastTime)%60 
				if (t > self.t_move):
					self.process = 1
					self.cmd_vel.linear.x = 0
					self.cmd_vel.angular.z = 0
					self.pub_vel.publish(self.cmd_vel)
					time.sleep(1)
					self.lastTime = time.time()
				else:
					self.cmd_vel.linear.x = self.vel_move
					self.cmd_vel.angular.z = 0
					self.pub_vel.publish(self.cmd_vel)

			elif self.process == 1: # rotaion
				t = (time.time() - self.lastTime)%60 
				if (t > self.t_rotation):
					self.process = 0
					self.cmd_vel.linear.x = 0
					self.cmd_vel.angular.z = 0
					self.pub_vel.publish(self.cmd_vel)
					time.sleep(1)					
					self.lastTime = time.time()
				else:
					self.cmd_vel.linear.x = 0
					self.cmd_vel.angular.z = self.vel_rotation
				self.pub_vel.publish(self.cmd_vel)

			self.rate.sleep()

		self.cmd_vel.linear.x = 0
		self.cmd_vel.angular.z = 0

		self.pub_vel.publish(self.cmd_vel)
		time.sleep(0.2)
		self.pub_vel.publish(self.cmd_vel)
		print('Programer stopped')


def main():
	print('Starting main program')

	program = testRun()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()	