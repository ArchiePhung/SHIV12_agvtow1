#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# DATE: 25/10/2021
# AUTHOR: HOANG VAN QUANG

import rospy
import sys
import time
import threading
import signal

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from message_pkg.msg import Driver_query
from message_pkg.msg import Driver_respond

from sti_msgs.msg import Velocities
from std_msgs.msg import Int16

from math import sin , cos , pi , atan2

class RPM:
	motor1 = 0
	motor2 = 0

class keep_pose():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('keep_pose', anonymous= False) # False
		self.rate = rospy.Rate(24)

		# -- parameter
		self.wheel_circumference = rospy.get_param("wheel_circumference", 0.47)
		self.transmission_ratio = rospy.get_param("transmission_ratio", 30.0)
		self.distanceBetwentWheels = rospy.get_param("distanceBetwentWheels", 0.541)
		self.frequency_control = rospy.get_param("frequency_control", 25.0)
		self.linear_max = rospy.get_param("linear_max", 0.8) # m/s
		self.linear_max = 0.3
		self.angular_max = rospy.get_param("angular_max", 0.5) # rad/s
		self.max_rpm = rospy.get_param("max_rpm", 3800)
		self.max_rpm = 100
		self.min_rpm = 100

		self.topicControl_vel =  rospy.get_param("topicControl_vel", "/cmd_vel") # 
		self.topicGet_vel =  rospy.get_param("topicGet_vel", "/raw_vel") # 

		self.topicControl_driverLeft = rospy.get_param("topicControl_driverLeft", "/driver1_query")
		self.topicControl_driverLeft = "/driver1_query"
		self.topicControl_driverRight = rospy.get_param("topicControl_driverRight", "/driver2_query")
		self.topicControl_driverRight = "/driver2_query"

		self.topicRespond_driverLeft = rospy.get_param("topicRespond_driverLeft", "/driver1_respond")
		self.topicRespond_driverLeft = "/driver1_respond"
		self.topicRespond_driverRight = rospy.get_param("topicRespond_driverRight", "/driver2_respond")
		self.topicRespond_driverRight = "/driver2_respond"

		self.frame_id = rospy.get_param("frame_id", "frame_robot")
		# -----------------
		rospy.Subscriber(self.topicControl_vel, Twist, self.cmdVel_callback)
		self.cmd_vel = Twist()

		rospy.Subscriber("/task_driver", Int16, self.taskDriver_callback)
		self.task_driver = Int16()

		rospy.Subscriber(self.topicRespond_driverLeft, Driver_respond, self.driver1Respond_callback)
		self.driver1_respond = Driver_respond()

		rospy.Subscriber(self.topicRespond_driverRight, Driver_respond, self.driver2Respond_callback)
		self.driver2_respond = Driver_respond()

		# -----------------
		self.pub_rawVel = rospy.Publisher(self.topicGet_vel, TwistWithCovarianceStamped, queue_size=50)
		self.raw_vel = TwistWithCovarianceStamped()

		self.pub_driverLeft = rospy.Publisher(self.topicControl_driverLeft, Driver_query, queue_size=50)
		self.driver1_query = Driver_query()

		self.pub_driverRight = rospy.Publisher(self.topicControl_driverRight, Driver_query, queue_size=50)
		self.driver2_query = Driver_query()
		self.driverRPM_query = RPM()

		self.lastTime_pub = time.time()
		self.time_pub = 1/self.frequency_control # s

		# -- 
		self.lastTime_speed1 = time.time()
		self.nowTime_speed1 = time.time()
		
		self.lastTime_speed2 = time.time()
		self.nowTime_speed2 = time.time()
		# -- frequence pub raw vel.
		self.isNew_driver1 = 0
		self.isNew_driver2 = 0
		# -- 
		self.is_exit = 1
		#-- 
		self.nowTime_cmdVel = time.time()
		# -- 
		self.max_deltaTime = 0.0
		# -- 
		self.max_rotation = 0.6 # rad/s
		# -- keep
		self.position1_now = 0.0
		self.timeStampe_pre1 = rospy.Time.now()
		self.position2_now = 0.0
		self.timeStampe_pre2 = rospy.Time.now()
		self.deceleration_distance = 0.02

	def taskDriver_callback(self, data):
		self.task_driver = data

	def cmdVel_callback(self, data):
		self.cmd_vel = data
		self.nowTime_cmdVel = time.time()


	def driver1Respond_callback(self, data):
		self.driver1_respond = data
		self.nowTime_speed1 = time.time()
		self.isNew_driver1 = 1
		# self.save_position(save_position.speed)
		
		# print ("reed")
		delta_t = (rospy.Time.now() - self.timeStampe_pre1).to_sec()
		self.timeStampe_pre1 = rospy.Time.now()

		if (delta_t < 0.1):
			self.position1_now += delta_t*self.convert_RPM_to_MPS(data.speed)
		else:
			print ("error1 !")
		# print ("position1_now: ", self.position1_now)

	def driver2Respond_callback(self, data):
		self.driver2_respond = data
		self.nowTime_speed2 = time.time()
		self.isNew_driver2 = 1

		# print ("read 2")
		delta_t = (rospy.Time.now() - self.timeStampe_pre2).to_sec()
		self.timeStampe_pre2 = rospy.Time.now()

		if (delta_t < 0.1):
			self.position2_now += delta_t*self.convert_RPM_to_MPS(data.speed)
		else:
			print ("error2 !")

	def constrain(self, val_in, val_compare1, val_compare2):
		val = 0.0
		val = max(val_in, val_compare1)
		val = min(val_in, val_compare2)
		return val

	def convert_RPM_to_MPS(self, rpm):
		mps = rpm/60./self.transmission_ratio*self.wheel_circumference
		return mps

	def convert_MPS_to_RPM(self, mps):
		out = (mps/self.wheel_circumference)*60.*self.transmission_ratio
		return out

	def calculateRPM(self, linear_x, angular_z):
		linear_x_right = linear_x
		angular_x_right = angular_z

		linear_vel_x_mins = 0.0
		angular_vel_z_mins = 0.0
		tangential_vel = 0.0
		x_rpm = 0.0
		tan_rpm = 0.0
		# -- limit
		if (abs(linear_x_right) > self.linear_max):
			if (linear_x_right >= 0):
				linear_x_right = self.linear_max
			else:
				linear_x_right = -self.linear_max

		if (abs(angular_x_right) > self.angular_max):
			if (angular_x_right >= 0):
				angular_x_right = self.angular_max
			else:
				angular_x_right = -self.angular_max

		# convert m/s to m/min
		linear_vel_x_mins = linear_x_right * 60
		# convert rad/s to rad/min
		angular_vel_z_mins = angular_x_right * 60
		tangential_vel = angular_vel_z_mins * (self.distanceBetwentWheels / 2)

		x_rpm = linear_vel_x_mins / self.wheel_circumference
		tan_rpm = tangential_vel / self.wheel_circumference

		rpm_query = RPM()

		# calculate for the target motor RPM and direction
		# front-left motor
		rpm_query.motor1 = (x_rpm - tan_rpm)*self.transmission_ratio
		rpm_query.motor1 = self.constrain(rpm_query.motor1, -self.max_rpm, self.max_rpm)
		# front-right motor
		rpm_query.motor2 = (x_rpm + tan_rpm)*self.transmission_ratio
		rpm_query.motor2 = self.constrain(rpm_query.motor2, -self.max_rpm, self.max_rpm)
		return rpm_query

	def save_position(self, speed_in):
		delta_t = (rospy.Time.now() - self.timeStampe_pre).to_sec()
		self.timeStampe_pre = rospy.Time.now()

		if (delta_t < 0.1):
			self.position1_now += delta_t*self.convert_RPM_to_MPS(speed_in)
		else:
			print ("error!")
		# print ("position1_now: ", self.position1_now)

	def run_keep(self):
		# -- ----------
		speed_write1 = 0.0
		delta_pos1 = self.position1_now

		if (abs(delta_pos1) > 0.005):
			coefficient1 = abs(delta_pos1)/0.03
			if (coefficient1 > 1.):
				coefficient1 = 1.
			speed_write1 = self.convert_MPS_to_RPM(self.linear_max*coefficient1)
			
			if (speed_write1 < self.min_rpm):
				speed_write1 = self.min_rpm

			if (speed_write1 > self.max_rpm):
				speed_write1 = self.max_rpm

		else :
			speed_write1 = 0

		if (delta_pos1 == 0):
			speed_write1 = 0
		elif (delta_pos1 > 0):
			speed_write1 = speed_write1*(-1)
		else:
			speed_write1 = speed_write1
		# --
		self.driver1_query.task = 0
		self.driver1_query.rotationSpeed = int(speed_write1)
		# self.driver1_query.accelerationTime = 
		# self.driver1_query.decelerationTime = 
		# self.driver1_query.torqueLimiting = 
		self.driver1_query.modeStop = 1
		# -- ----------
		speed_write2 = 0.0
		delta_pos2 = self.position2_now

		if (abs(delta_pos2) > 0.005):
			coefficient2 = abs(delta_pos2)/0.03
			if (coefficient2 > 1.):
				coefficient2 = 1.
			speed_write2 = self.convert_MPS_to_RPM(self.linear_max*coefficient2)
			
			if (speed_write2 < self.min_rpm):
				speed_write2 = self.min_rpm

			if (speed_write2 > self.max_rpm):
				speed_write2 = self.max_rpm

		else :
			speed_write2 = 0

		if (delta_pos2 == 0):
			speed_write2 = 0
		elif (delta_pos2 > 0):
			speed_write2 = speed_write2*(-1)
		else:
			speed_write2 = speed_write2
		# --
		self.driver2_query.task = 0
		self.driver2_query.rotationSpeed = int(speed_write2)
		# self.driver2_query.accelerationTime = 
		# self.driver2_query.decelerationTime = 
		# self.driver2_query.torqueLimiting = 
		self.driver2_query.modeStop = 1
		# --
		self.pub_driverLeft.publish(self.driver1_query)
		self.pub_driverRight.publish(self.driver2_query)

	def run(self):
		print ("Launch ALL!")
		while not rospy.is_shutdown():
			self.run_keep()

			self.rate.sleep()
		self.is_exit = 0
		print('program stopped')


def main():
	print('Starting main program')

	# Start the job threads
	programer = keep_pose()
	programer.run()

	print('Exiting main program')

if __name__ == '__main__':
	main()