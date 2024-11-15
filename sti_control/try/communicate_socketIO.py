#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date: 28/06/2023
"""

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


class Communicate_socketIO():
	def __init__(self):
		rospy.init_node('Communicate_socketIO', anonymous=False)
		self.rate = rospy.Rate(10)
		# -- 
		self.name_card = rospy.get_param("name_card", "wlp0s20f3")
		self.name_card = "wlo2" # "wlp0s20f3"
		self.server_IP = '192.168.1.99'
		self.server_port = 4001

		self.AGV_IP = '192.168.1.100'
		self.AGV_port = 6000
		self.AGV_mac = "0c:9a:3c:07:bb:6f"

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
		print ("Freq: ", 1.0/delta_t.to_sec())

	def run(self):

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
		# print('I received a message!')
		data_json = json.loads(data)
		# print (data_json['mac'])
		if data_json['mac'] == myObject.AGV_mac:
			# print ("Server request info to Me!")
			myObject.measureFreq_event()

			x = round(myObject.AGV_information.x, 3)
			y = round(myObject.AGV_information.y, 3)
			r = round(myObject.AGV_information.z, 3)
			status = myObject.AGV_information.status
			battery = myObject.AGV_information.battery
			mode = myObject.AGV_information.mode
			listErrors = myObject.AGV_information.listError

			data_send = {"id": data_json['id'], "name": data_json['name'], "mac": data_json['mac'], "mode": mode, "status": 1, "x": x, "y": y, "r": r, "status": status, "battery": battery, "listErrors": listErrors}
			my_socketIO.emit("AGV-respond-info", json.dumps(data_send, indent = 4))

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

if __name__ == '__main__':
    main()