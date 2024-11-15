#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date: 28/06/2023
"""

import random
import socketio
import json 

import socket
import roslib

# get ip
import os                                                                                                                                                           
import re 

import sys
import struct
import time
from decimal import *
import math
import rospy
from datetime import datetime

# from sti_msgs.msg import NN_cmdRequest   
# from sti_msgs.msg import NN_infoRequest  
# from sti_msgs.msg import NN_infoRespond 

# from sti_msgs.msg import FL_infoRespond
# from sti_msgs.msg import FL_cmdRespond
# from sti_msgs.msg import FL_infoRequest
# from sti_msgs.msg import FL_cmdRequest

# import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

class Communicate_socketIO():
	def __init__(self):
		rospy.init_node('sim_robot14', anonymous=False)
		self.rate = rospy.Rate(8)
		# -- 
		self.name_card = rospy.get_param("name_card", "wlp0s20f3")
		self.name_card = "wlo2" # "wlp0s20f3"
		self.server_IP = '192.168.1.99'
		# self.server_IP = '127.0.0.1'
		self.server_port = 4014

		self.AGV_IP = '192.168.1.100'
		self.AGV_port = 6004
		self.AGV_mac = "0c:9a:3c:07:bb:74"
		# -
		self.robot_x = 0.0
		self.robot_y = 0.0
		self.robot_r = radians(45)
		self.timeD = 1./8.
		# -
		self.vel_x = 0.15
		self.vel_r = 0.0
		self.step = 0
		# -
		# rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.AGVInfor_callback)	
		# self.AGV_information = NN_infoRespond()
		# self.is_AGVInfor = 0
		# -
		# self.NN_cmdPub = rospy.Publisher("/NN_cmdRequest", NN_cmdRequest, queue_size=50)
		# self.server_sendCommand = NN_cmdRequest()
		# # -
		# self.NN_infoRequestPub = rospy.Publisher("/NN_infoRequest", NN_infoRequest, queue_size=50)
		# self.server_requestInfor = NN_infoRequest()
		# ---------
		self.saveTime_received = rospy.Time.now()

	def AGVInfor_callback(self, data):
		self.AGV_information = data
		self.is_AGVInfor = 1

	def measureFreq_event(self):
		delta_t = rospy.Time.now() - self.saveTime_received
		self.saveTime_received = rospy.Time.now()
		# print ("Freq: ", 1.0/delta_t.to_sec())

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def limitAngle(self, angle_in): # - rad
		qua_in = self.euler_to_quaternion(angle_in)
		angle_out = self.quaternion_to_euler(qua_in)
		return angle_out

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat
	
	def move(self, vel_x, vel_r):
		delta_r = vel_r*self.timeD
		delta_x = vel_x*cos(self.robot_r)*self.timeD
		delta_y = vel_x*sin(self.robot_r)*self.timeD

		# -----------
		self.robot_x += delta_x
		self.robot_y += delta_y
		self.robot_r += delta_r
		self.robot_r = self.limitAngle(self.robot_r)

	def run(self):
		self.move(0.4, 0.2)

		# self.move(self.vel_x, self.vel_r)
		self.rate.sleep()

def main():
	print('Starting main program')
    # - Start the job threads
	myObject = Communicate_socketIO()		
	my_socketIO = socketio.Client()

	# my_socketIO.connect('http://' + myObject.server_IP +':' + str(myObject.server_port))
	# -
	is_connected = 0
	time_save = time.time()

	@my_socketIO.on('Server-request-agv-info')
	def on_message(data):
		# print('I received Request form Server!')

		data_json = json.loads(data)
		# print (data_json['mac'])
		if data_json['mac'] == myObject.AGV_mac:
			print ("Server request: Info to Me!")
			myObject.measureFreq_event()

			x = round(myObject.robot_x, 3)
			y = round(myObject.robot_y, 3)
			r = round(myObject.robot_r, 3)
			battery = round(random.uniform(23.0, 25.5), 1)
			status = round(random.uniform(0, 2), 0)
			# print ("x: ", x)
			# x = round(myObject.AGV_information.x, 3)
			# y = round(myObject.AGV_information.y, 3)
			# r = round(myObject.AGV_information.z, 3)
			# status = myObject.AGV_information.status
			# battery = myObject.AGV_information.battery

			mode = 0# myObject.AGV_information.mode
			listErrors = [] # myObject.AGV_information.listError

			data_send = {"id": data_json['id'], "name": data_json['name'], "mac": data_json['mac'], "mode": mode, "status": 1, "x": x, "y": y, "r": r, "status": status, "battery": battery, "listErrors": listErrors}
			my_socketIO.emit("Robot-respond-info", json.dumps(data_send, indent = 4))

	@my_socketIO.on('Server-send-agv-cmd')
	def on_message(data):
		# data_json = json.loads(data)
		# if data_json['mac'] == myObject.AGV_mac:
		print ("Server send: Command to Me!")

	@my_socketIO.event
	def connect():
		print("I'm connected!")
		is_connected = 1

	@my_socketIO.event
	def connect_error(data):
		print("The connection failed!")

	@my_socketIO.event 
	def disconnect():
		print("I'm disconnected!")

	# - Keep the main thread running, otherwise signals are ignored.
	while not rospy.is_shutdown():
		myObject.run()
		# -
		if is_connected == 0:
			delta_t = (time.time() - time_save)%60
			if delta_t > 0.6:
				time_save = time.time()
				try:
					my_socketIO.connect('http://' + myObject.server_IP +':' + str(myObject.server_port))
				except Exception:
					pass
	my_socketIO.disconnect()

if __name__ == '__main__':
    main()
