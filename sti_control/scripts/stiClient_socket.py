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

from sti_msgs.msg import NN_cmdRequest   
from sti_msgs.msg import NN_infoRequest  
from sti_msgs.msg import NN_infoRespond 

from sti_msgs.msg import FL_infoRespond
from sti_msgs.msg import FL_cmdRespond
from sti_msgs.msg import FL_infoRequest
from sti_msgs.msg import FL_cmdRequest

# import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sti_msgs.msg import *
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees


class Communicate_socketIO():
	def __init__(self):
		rospy.init_node('stiClient_socket', anonymous=False)
		self.rate = rospy.Rate(4)
		# -- 
		self.server_IP = '192.168.0.26'
		# self.server_IP = '127.0.0.1'

		self.server_port = 4001
		self.AGV_mac = "6c:6a:77:df:90:6c"
		# -
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.AGVInfor_callback)	
		self.AGV_information = NN_infoRespond()
		self.is_AGVInfor = 0
		# -
		self.NN_cmdPub = rospy.Publisher("/NN_cmdRequest", NN_cmdRequest, queue_size=50)
		self.server_sendCommand = NN_cmdRequest()
		# -
		self.NN_infoRequestPub = rospy.Publisher("/NN_infoRequest", NN_infoRequest, queue_size=50)
		self.server_requestInfor = NN_infoRequest()
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

	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)

	def Navi_cmdAnalysis(self, data):
		self.server_sendCommand.target_id = data['command']['target']['id']
		# self.server_sendCommand.target_x = data['command']['target']['x']
		# self.server_sendCommand.target_y = data['command']['target']['y']
		# self.server_sendCommand.target_z = data['command']['target']['r']
		# self.server_sendCommand.before_mission = data['command']['job']['before']
		# self.server_sendCommand.after_mission = data['command']['job']['after']
		# self.server_sendCommand.commandStop = data['command']['requirStop']
	
	def run(self):
		self.NN_cmdPub.publish(self.server_sendCommand)
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

	@my_socketIO.on('Server-query-agv-info')
	def on_message(data):
		# print('I received Request form Server!')
		data_json = json.loads(data)
		if data_json['mac'] == myObject.AGV_mac:
			print ("Server request: Info to Me!")
			myObject.measureFreq_event()
			# -
			x = round(myObject.AGV_information.x, 3)
			y = round(myObject.AGV_information.y, 3)
			r = round(myObject.AGV_information.z, 3)
			status = myObject.AGV_information.status
			battery = myObject.AGV_information.battery
			mode = myObject.AGV_information.mode
			tag = myObject.AGV_information.tag
			offset = myObject.AGV_information.offset
			listErrors = myObject.AGV_information.listError

			data_send = {"id": data_json['id'], "name": data_json['name'], "mac": data_json['mac'], "x": x, "y": y, "r": r, "mode": mode, "status": status, "voltage": battery, "tag": tag, "offset": offset, "listErrors": listErrors, "isLoad": 1}
			my_socketIO.emit("Robot-respond-info", json.dumps(data_send, indent = 4))
			print ("data_send: ", data_send)

	@my_socketIO.on('Server-send-agv-cmd')
	def on_message(data):
		# data_json = json.loads(data)
		# if data_json['mac'] == myObject.AGV_mac:
		# print ("Server send: Command to Me: ", data)
		myObject.Navi_cmdAnalysis(json.loads(data))

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
