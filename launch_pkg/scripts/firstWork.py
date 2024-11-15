#!/usr/bin/env python3
# Author : Hoang Van Quang - BEE
# Date: 11-06-2020

import os
import rospy
import time
from std_msgs.msg import Int16

class WorkFirst:
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('first_work', anonymous= True)
		self.rate = rospy.Rate(10)

		self.pub_run = rospy.Publisher('/first_work/run', Int16, queue_size= 10)
		self.run = Int16()
		self.count = 0

	def doIt(self):
		os.system("yes | rosclean purge") # xoa log ROS
		while not rospy.is_shutdown():

			self.pub_run.publish(self.run)
			self.count += 1
			if (self.count > 100):
				break

			self.rate.sleep()

def main():
	print('Program starting')

	program = WorkFirst()
	program.doIt()

	print('Programer stopped')

if __name__ == '__main__':
    main()