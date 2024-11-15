#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: HOANG VAN QUANG - BEE
# DATE: 23/06/2021

from sti_msgs.msg import Zone_lidar_2head
from sti_msgs.msg import Ps2_msgs
from sti_msgs.msg import Velocities
from geometry_msgs.msg import Twist
import time
import rospy

class naviManual():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('navi_manual', anonymous=False)
		self.rate = rospy.Rate(25)

		rospy.Subscriber("/ps2_status", Ps2_msgs, self.ps2_callback)
		self.ps2_status = Ps2_msgs()

		rospy.Subscriber("/zone_safety", Zone_lidar_2head, self.ps2_callback)
		self.zone_lidar = Zone_lidar_2head()

		self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size= 10)

		self.linear_lv1 = 0.6 # m/s
		self.angular_lv1 = 0.4

		self.linear_lv2 = 0.4
		self.angular_lv2 = 0.2

		self.nowTime = time.time()
		self.timeout = 0.4

	def ps2_callback(self, dat):
		self.ps2_status = dat
		self.nowTime = time.time()

	def run(self):
	    while not rospy.is_shutdown():
			vel_ps2 = Twist()
			vel_zone = Twist()
			# -- ps2
			t = time.time() - self.nowTime
			if (t < self.timeout):
				if self.ps2_status.up.data == 1 and self.ps2_status.down.data == 0:   # Fast 
					if (self.ps2_status.triangle.data == 1) and (self.ps2_status.circle.data == 0) and (self.ps2_status.cross.data == 0) and (self.ps2_status.square.data == 0): # forward
						vel_ps2.linear.x = linear_lv1
						vel_ps2.angular.z = 0.0

					elif (self.ps2_status.triangle.data == 0) and (self.ps2_status.circle.data == 0) and (self.ps2_status.cross.data == 1) and (self.ps2_status.square.data == 0): # backward
						vel_ps2.linear.x = -linear_lv1
						vel_ps2.angular.z = 0.0

					elif (self.ps2_status.triangle.data == 0) and (self.ps2_status.circle.data == 0) and (self.ps2_status.cross.data == 0) and (self.ps2_status.square.data == 1): # turn left
						vel_ps2.linear.x = 0.0
						vel_ps2.angular.z = angular_lv1
					elif (self.ps2_status.triangle.data == 0) and (self.ps2_status.circle.data == 1) and (self.ps2_status.cross.data == 0) and (self.ps2_status.square.data == 0): # turn right
						vel_ps2.linear.x = 0.0
						vel_ps2.angular.z = -angular_lv1
					else:
						vel_ps2.linear.x = 0.0
						vel_ps2.angular.z = 0.0

				elif self.ps2_status.up.data == 0 and self.ps2_status.down.data == 1:  # Slow
					if (self.ps2_status.triangle.data == 1) and (self.ps2_status.circle.data == 0) and (self.ps2_status.cross.data == 0) and (self.ps2_status.square.data == 0): # forward
						vel_ps2.linear.x = linear_lv2
						vel_ps2.angular.z = 0.0

					elif (self.ps2_status.triangle.data == 0) and (self.ps2_status.circle.data == 0) and (self.ps2_status.cross.data == 1) and (self.ps2_status.square.data == 0): # backward
						vel_ps2.linear.x = -linear_lv2
						vel_ps2.angular.z = 0.0

					elif (self.ps2_status.triangle.data == 0) and (self.ps2_status.circle.data == 0) and (self.ps2_status.cross.data == 0) and (self.ps2_status.square.data == 1): # turn left
						vel_ps2.linear.x = 0.0
						vel_ps2.angular.z = angular_lv2
					elif (self.ps2_status.triangle.data == 0) and (self.ps2_status.circle.data == 1) and (self.ps2_status.cross.data == 0) and (self.ps2_status.square.data == 0): # turn right
						vel_ps2.linear.x = 0.0
						vel_ps2.angular.z = -angular_lv2
					else:
						vel_ps2.linear.x = 0.0
						vel_ps2.angular.z = 0.0
				else:
					vel_ps2.linear.x = 0.0
					vel_ps2.angular.z = 0.0
			else:
				vel_ps2.linear.x = 0.0
				vel_ps2.angular.z = 0.0
			
			# ---- zone safety
			if (vel_ps2.linear.x > 0 and vel_ps2.angular.z == 0):
				if (self.zone_lidar.zone_ahead == 0):
					vel_zone.linear.x = vel_ps2.linear.x

				elif (self.zone_lidar.zone_ahead == 3):
					vel_zone.linear.x = vel_ps2.linear.x*0.8

				elif (self.zone_lidar.zone_ahead == 2):
					vel_zone.linear.x = vel_ps2.linear.x*0.6

				elif (self.zone_lidar.zone_ahead == 1):
					vel_zone.linear.x = 0.0

			if (vel_ps2.linear.x < 0 and vel_ps2.angular.z == 0):
				if (self.zone_lidar.zone_behind == 0):
					vel_zone.linear.x = vel_ps2.linear.x

				elif (self.zone_lidar.zone_behind == 3):
					vel_zone.linear.x = vel_ps2.linear.x*0.8

				elif (self.zone_lidar.zone_behind == 2):
					vel_zone.linear.x = vel_ps2.linear.x*0.6

				elif (self.zone_lidar.zone_behind == 1):
					vel_zone.linear.x = 0.0

			elif (vel_ps2.linear.x == 0 and vel_ps2.angular.z != 0):
				vel_zone = vel_ps2

			elif (vel_ps2.linear.x != 0 and vel_ps2.angular.z != 0):
				vel_zone.linear.x = 0.0
				vel_zone.angular.z = 0.0

			elif (vel_ps2.linear.x == 0 and vel_ps2.angular.z == 0):
				vel_zone.linear.x = 0.0
				vel_zone.angular.z = 0.0

			self.pub_vel.publish(vel_zone)
            self.rate.sleep()

def main():
    print('Program starting')
    try:
        program = naviManual()
        program.run()
    except rospy.ROSInterruptException:
        pass
    print('Programer stopped')

if __name__ == '__main__':
    main()