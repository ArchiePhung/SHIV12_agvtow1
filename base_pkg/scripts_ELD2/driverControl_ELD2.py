#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#Authors : BEE
# blv 200W.
# DATE: 15/03/2021
# AUTHOR: HOANG VAN QUANG

import rospy
import sys
import time
import threading
import signal

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from message_pkg.msg import Driver_query
from message_pkg.msg import Driver_respond

from std_msgs.msg import Bool

from math import sin , cos , pi , atan2

# from pykalman import KalmanFilter

import serial
import modbus_tk # pip install modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

# kết nối khi khởi động.
# Nhận diện mất kết nối.
# Tự động kết nối lại.
class statusDriver:
	def __init__(self, id):
		self.ID = id
		self.FWD = 0
		self.REV = 0
		self.STOP_MODE = 0
		self.WNG = 0
		self.ALARM_OUT1 = 0
		self.S_BSY = 0
		self.ALARM_OUT2 = 0	
		self.MOVE = 0
		self.VA = 0
		self.TLC = 0
		self.PRESENT_ALARM = 0
		self.PRESENT_WARNING = 0
		self.SPEED = 0
		self.POSITION = 0
		self.SPEED2 = 0

	def showAll(self):
		print ("FWD | REV | STOP_MODE | WNG | ALARM_OUT1 | S_BSY | ALARM_OUT2 | MOVE | VA | TLC | PRESENT_ALARM | PRESENT_WARNING | SPEED")
		print (" " + str(self.FWD) + "  |  " + str(self.REV) + "  |      " + str(self.STOP_MODE) + "    |  " \
		+ str(self.WNG) + "  |      " + str(self.ALARM_OUT1) + "     |   " + str(self.S_BSY) + "   |      " \
		+ str(self.ALARM_OUT2) + "     |   "  + str(self.MOVE) + "  |  "  + str(self.VA) \
		+ " |  "  + str(self.TLC) + "  |        "  + str(self.PRESENT_ALARM) + "      |         "  + str(self.PRESENT_WARNING)\
		+ "       |   "  + str(self.SPEED))

class controlDriver:
	def __init__(self, id = 0, rev = 0):
		self.ID = id
		self.FWD = 0
		self.REV = 0
		self.STOP_MODE = 0
		self.SPEED = 0
		self.ACCELERATION_TIME = 1
		self.DECELERATION_TIME = 1
		self.TORQUE_LIMITING = 200
		self.REVERT = rev # Đảo ngược quy định chiều động cơ.

class driver():
	def __init__(self):
		# status = 1 (all right) | != 1 error.

		self.PORT =  rospy.get_param("port", "/dev/ttyUSB0")
		self.BAUDRATE = rospy.get_param("baudrate", 57600) # 57600 115200
		self.maxRPM = rospy.get_param("maxRPM", 3800)
		self.minRPM = self.maxRPM*(-1)
		self.topicSafety = rospy.get_param("topicSafety", "/safety")
		# -------------------
		self.ID_Driver_1 = rospy.get_param("id_1", 1)
		self.topicSub_1 = rospy.get_param("topicSub_1", "/driver1_query")
		self.topicPub_1 = rospy.get_param("topicPub_1", "/driver1_respond")
		self.revert_1 = rospy.get_param("revert_1", 0)
		# -------------------
		self.ID_Driver_2 = rospy.get_param("id_2", 2)
		self.topicSub_2 = rospy.get_param("topicSub_2", "/driver2_query")
		self.topicPub_2 = rospy.get_param("topicPub_2", "/driver2_respond")
		self.revert_2 = rospy.get_param("revert_2", 0)

		self.lastTime_readStatus = time.time()
		self.time_readStatus = 2 # s
		# -- 
		self.perimeter = 0.47
		self.transmission_ratio = 30

		self.accelerationRate = 1000 # [1 to 1,000,000] ms
		self.decelerationRate = 200 # [1 to 1,000,000] ms
		self.torqueLimiting = 200 # 200    # 0 to 10,000 (1=0.1%)
		# -- 
		self.reg_communicationTimeout = 5003 # P222
		self.reg_CommunicationErrorDetection = 5005 # 
		# -- Registers
		self.reg_operationData = 89 # (0059h) - [0 to 255] P66
		self.reg_operationType = 91 #
		self.reg_operatingVelocity = 95 #
		self.reg_accelerationRate = 97  #
		self.reg_decelerationRate = 99  #
		self.reg_torqueLimiting = 101   # 0 to 10,000 (1=0.1%) - P67
		self.reg_operationTrigger = 103 #
		# -- 
		self.reg_inputCommandUpper = 124 #
		self.reg_inputCommandLower = 125 #

		self.reg_outputStatusUpper = 126 #
		self.reg_outputStatusLower = 127 #
		# -- 
		self.reg_presentAlarm = 129  # - P304
		self.reg_feedbackSpeed = 207 # - P305           # r/m
		self.reg_feedbackPosition = 289                 # step
		self.reg_feedbackSpeed2 = 209                   # step/s

		self.reg_resentCommunicationError = 173 # P304
		self.reg_driverTemperature = 249 # P306
		
		self.reg_supplyVoltage = 329 # (1=0.1 V) P308
		self.reg_resetAlarm = 385 # P302
		self.reg_stopOperation = 447 # 
		self.reg_resetCommunication = 445 #
		# - 
		self.reg_operationDataNo0_type = 6145 # P326
		self.reg_operationDataNo0_position = 6147 #
		self.reg_operationDataNo0_velocity = 6149 # P326
		self.reg_operationDataNo0_accelerationRate = 6151 # P326
		self.reg_operationDataNo0_decelerationRate = 6153 # P326
		self.reg_operationDataNo0_torqueLimiting = 6155 #
		self.reg_operationDataNo0_accelerationTime = 6157 #
		self.reg_operationDataNo0_decelerationTime = 6159 #
		# -
		self.reg_operationDataNo2_type = 6273 # P326
		self.reg_operationDataNo2_position = 6275 #
		self.reg_operationDataNo2_velocity = 6277 # P326
		self.reg_operationDataNo2_accelerationRate = 6279 # P326
		self.reg_operationDataNo2_decelerationRate = 6281 # P326
		self.reg_operationDataNo2_torqueLimiting = 6283 #
		self.reg_operationDataNo2_accelerationTime = 6285 #
		self.reg_operationDataNo2_decelerationTime = 6287 #
		# -- 
		self.reg_driverOutputCommand_upper = 126 # 0x7F - pape 225
		self.reg_driverOutputCommand_lower = 127 # 0x7F - pape 225

		# --------
		self.bitInputCommand_FW_JOG = 0
		self.bitInputCommand_FV_JOG = 1
		self.bitInputCommand_FW_SPD = 2
		self.bitInputCommand_FV_SPD = 3
		self.bitInputCommand_HOME = 4
		self.bitInputCommand_NOT5 = 5
		self.bitInputCommand_START = 6
		self.bitInputCommand_SSTART = 7
		self.bitInputCommand_M0 = 8
		self.bitInputCommand_M1 = 9
		self.bitInputCommand_M2 = 10
		self.bitInputCommand_M3 = 11
		self.bitInputCommand_M4 = 12
		self.bitInputCommand_M5 = 13
		self.bitInputCommand_M6 = 14
		self.bitInputCommand_M7 = 15
		# -- 
		self.bitInputCommand_S_ON = 0
		self.bitInputCommand_PLOOP_MODE = 1
		self.bitInputCommand_TRQ_LMT = 2
		self.bitInputCommand_CLR   = 3
		self.bitInputCommand_QSTOP = 4
		self.bitInputCommand_STOP  = 5
		self.bitInputCommand_FREE  = 6
		self.bitInputCommand_ALM_RST = 7
		self.bitInputCommand_D_SEL0  = 8
		self.bitInputCommand_D_SEL1  = 9
		self.bitInputCommand_D_SEL2  = 10
		self.bitInputCommand_D_SEL3  = 11
		self.bitInputCommand_D_SEL4  = 12
		self.bitInputCommand_D_SEL5  = 13
		self.bitInputCommand_D_SEL6  = 14
		self.bitInputCommand_D_SEL7  = 15
		# --
		self.bitOutputStatus_INFO     = 0
		self.bitOutputStatus_INFO_MNT = 1
		self.bitOutputStatus_DRVTMP   = 2
		self.bitOutputStatus_MTRTMP   = 3
		self.bitOutputStatus_INFO_TRQ = 4
		self.bitOutputStatus_INFO_WATT   = 5
		self.bitOutputStatus_INFO_VOLTH  = 6
		self.bitOutputStatus_INFO_VOLTL  = 7
		self.bitOutputStatus_CONST_OFF24 = 8
		self.bitOutputStatus_CONST_OFF25 = 9
		self.bitOutputStatus_CONST_OFF26 = 10
		self.bitOutputStatus_CONST_OFF27 = 11
		self.bitOutputStatus_CONST_OFF28 = 12
		self.bitOutputStatus_CONST_OFF29 = 13
		self.bitOutputStatus_USR_OUT0    = 14
		self.bitOutputStatus_USR_OUT1    = 15
		# --
		self.bitOutputStatus_SON_MON   = 0
		self.bitOutputStatus_PLOOP_MON = 1
		self.bitOutputStatus_TRQ_LMTD  = 2
		self.bitOutputStatus_RDY_DD = 3
		self.bitOutputStatus_ABSPEN = 4
		self.bitOutputStatus_STOP_R = 5
		self.bitOutputStatus_FREE_R = 6
		self.bitOutputStatus_ALM_A  = 7
		self.bitOutputStatus_SYS_BSY  = 8
		self.bitOutputStatus_IN_POS   = 9
		self.bitOutputStatus_RDY_HOME = 10
		self.bitOutputStatus_RDY_FWRV = 11
		self.bitOutputStatus_RDY_SD   = 12
		self.bitOutputStatus_MOVE = 13
		self.bitOutputStatus_VA   = 14
		self.bitOutputStatus_TLC  = 15
		# -------------------------------------------
		# -- 
		self.statusDriver_1 = statusDriver(self.ID_Driver_1)
		self.statusDriver_2 = statusDriver(self.ID_Driver_2)
		self.controlDriver_1 = controlDriver(self.ID_Driver_1, self.revert_1)
		self.controlDriver_2 = controlDriver(self.ID_Driver_2, self.revert_2)

		# ---------------------------------------- Modbus
		print("Modbus run!")
		for i in range(3):
			try:
				self.MODBUS = modbus_rtu.RtuMaster(
					serial.Serial(port= self.PORT, baudrate= self.BAUDRATE, bytesize=8, parity='E', stopbits=1, xonxoff=0)
				)

				self.MODBUS.open 
				self.MODBUS.set_timeout(1.0)
				self.MODBUS.set_verbose(True)
				print("Modbus connected !")
				break

			except modbus_tk.modbus.ModbusError as exc:
				print("Modbus false!") 
				sys.exit()
		# ---------------------------------------- ROS
		print("ROS Initial!")
		rospy.init_node('driver_control', anonymous=False)
		self.rate = rospy.Rate(200)

		rospy.Subscriber(self.topicSafety, Bool, self.safety_infoCallback)
		self.safety = Bool()
		self.enable_run = 0
		self.is_readSafety = 0
		self.lastTime_checkSafety = time.time()
		self.cycle_checkSafety = 1 # s

		# ------------- SUB ------------- #
		rospy.Subscriber("/disable_brake", Bool, self.break_infoCallback)
		self.disable_brake = Bool()

		rospy.Subscriber(self.topicSub_1, Driver_query, self.driverQuery1_infoCallback)
		self.driverQuery_1 = Driver_query()
		self.is_readQuery_1 = 0
		self.lastTime_checkQuery_1 = time.time()
		self.cycle_checkQuery = 0.5 # s

		rospy.Subscriber(self.topicSub_2, Driver_query, self.driverQuery2_infoCallback)
		self.driverQuery_2 = Driver_query()
		self.is_readQuery_2 = 0
		self.lastTime_checkQuery_2 = time.time()

		# ------------- PUB ------------- #
		self.pub_driverRespond_1 = rospy.Publisher(self.topicPub_1, Driver_respond, queue_size= 50)
		self.driverRespond_1 = Driver_respond()

		self.pub_driverRespond_2 = rospy.Publisher(self.topicPub_2, Driver_respond, queue_size= 50)
		self.driverRespond_2 = Driver_respond()

		self.status_error = 2
		self.status_run = 1
		self.status_stop = 0
		# -- 
		self.timeWait = 0.004
		# -- 
		self.frequence_pubStatus = 25.
		self.cycle_pubStatus = 1/self.frequence_pubStatus
		self.preTime_pubStatus = time.time()
		# --
		self.countErr_reqSpeed = 0
		self.step = 1

	def break_infoCallback(self, data):
		self.disable_brake = data

	def driverQuery1_infoCallback(self, data):
		self.driverQuery_1 = data
		self.is_readQuery_1 = 1
		self.lastTime_checkQuery_1 = time.time()

	def driverQuery2_infoCallback(self, data):
		self.driverQuery_2 = data
		self.is_readQuery_2 = 1
		self.lastTime_checkQuery_2 = time.time()

	def safety_infoCallback(self, data):
		self.safety = data
		self.is_readSafety = 1
		self.lastTime_checkSafety = time.time()

# ------------------------------------------------------
	def mean_ALM_A(self, err):
		mess = ' '
		switcher={
			0:   'All right!', # 
			16: 'Position deviation (300 rev) | L7', # 10h
			33: 'Main circuit overheat > 85°C | L7', # 21h
			34: 'Overvoltage > 63 V | L5', # 22h
			37: 'Undervoltage < 14 V | L5', # 25h
			38: 'Motor overheat 95°C | L7', # 26h
			49: 'Overspeed | L7', # 31h
			# --
			32: 'Overcurrent | L9', # 20h
			40: 'Encoder error | L2', # 28h
			41: 'Internal circuit error: CPU peripheral | L9', # 29h
			42: 'Encoder communication error | L2', # 2Ah
			48: 'Overload | L7', # 30h
			65: 'EEPROM error | L9', # 41h
			66: 'Initial encoder error L2', # 42h
			68: 'Encoder EEPROM error', # 44h
			69: 'Motor combination error', # 45h
			74: 'Homing incomplete', # 4Ah
			80: 'Electromagnetic brake overcurrent | L9', # 50h
			83: 'HWTO input circuit error', # 53h
			85: 'The electromagnetic brake connection error', # 55h
			96: '±LS both sides active', # 60h
			97: 'Reverse ±LS connection', # 61h
			98: 'Homing operation error', # 62h
			99: 'No HOMES', # 63h
			100: 'Z, SLIT signal error', # 64h
			102: 'Hardware overtravel', # 66h
			103: 'Software overtravel', # 67h
			104: 'HWTO input detection', # 68h
			106: 'Homing additional operation error', # 6Ah
			112: 'Operation data error', # 70h
			113: 'Unit setting error', # 71h
			129: 'Network bus error', # 81h
			132: 'RS-485 communication error', # 84h
			133: 'RS-485 communication timeout', # 85h
			140: 'Out of setting range', # 8Ch
			240: 'CPU error', # F0h
			243: 'CPU overload' # F3h
		}
		return switcher.get(err, 'Not found!')

	def resetAlarm(self, id): # - OK
		rawData = 0
		# ---------------------------------
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_resetAlarm, output_value = 1)
		except modbus_tk.modbus.ModbusError as exc:
			print ("Write reg_resetAlarm ON Error ID" + str(id))
			self.countErr_reqSpeed += 1
		# ---------------------------------
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_resetAlarm, output_value = 0)
		except modbus_tk.modbus.ModbusError as exc:
			print ("Write reg_resetAlarm OFF Error ID" + str(id))
			self.countErr_reqSpeed += 1

	def resetAll(self):  # - OK
		print ("Run reset")
		if (self.statusDriver_1.WNG == 1 or self.statusDriver_1.ALARM_OUT1 == 1 or self.statusDriver_1.ALARM_OUT2 == 1 or self.statusDriver_1.PRESENT_ALARM != 0):
			self.resetAlarm(self.controlDriver_1.ID)
			print ("reset ID_Driver_1")
			
		if (self.statusDriver_2.WNG == 1 or self.statusDriver_2.ALARM_OUT1 == 1 or self.statusDriver_2.ALARM_OUT2 == 1 or self.statusDriver_2.PRESENT_ALARM != 0):
			self.resetAlarm(self.controlDriver_2.ID)
			print ("reset ID_Driver_2")

	def getSpeed_all(self): # - OK
		# -- Driver 1
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.READ_HOLDING_REGISTERS, self.reg_feedbackSpeed, 1)
			spd = 0
			if (rawData[0] > 6000):
				spd = rawData[0] - pow(2, 16)
			else:
				spd = rawData[0]

			if (self.revert_1 == 1):
				self.statusDriver_1.SPEED = -spd
			else:
				self.statusDriver_1.SPEED = spd

		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_feedbackSpeed ID1 error")
			self.countErr_reqSpeed += 1

		# -- Driver 2
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.READ_HOLDING_REGISTERS, self.reg_feedbackSpeed, 1)
			spd = 0
			if (rawData[0] > 6000):
				spd = rawData[0] - pow(2, 16)
			else:
				spd = rawData[0]

			if (self.revert_2 == 1):
				self.statusDriver_2.SPEED = -spd
			else:
				self.statusDriver_2.SPEED = spd

		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_feedbackSpeed ID2 error")
			self.countErr_reqSpeed += 1

	def getPosition_all(self):
		# -- Driver 1
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.READ_HOLDING_REGISTERS, self.reg_feedbackPosition, 1)
			print("Position ID1 is:", rawData[0])
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_feedbackPosition ID1 error")
			self.countErr_reqSpeed += 1

		# -- Driver 2
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.READ_HOLDING_REGISTERS, self.reg_feedbackPosition, 1)
			# print("Position ID2 is:", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_feedbackPosition ID2 error")
			self.countErr_reqSpeed += 1


	def getSpeed2_all(self):
		# -- Driver 1
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.READ_HOLDING_REGISTERS, self.reg_feedbackSpeed2, 1)
			print("Speed ID1 is:", rawData[0])
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_feedbackSpeed2 ID1 error")
			self.countErr_reqSpeed += 1

		# -- Driver 2
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.READ_HOLDING_REGISTERS, self.reg_feedbackSpeed2, 1)
			print("Speed ID2 is:", rawData[0])
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_feedbackSpeed2 ID2 error")
			self.countErr_reqSpeed += 1

	def getStatus_all(self, type_): # - OK
		# -- get vel, alarm, warning, error, ...
		rawData = 0
		allData = ''
		t = (time.time() - self.lastTime_readStatus)%60 

		# ----------------- Driver 1 ----------------- #
		if (t > self.time_readStatus or type_ == 1):
			self.lastTime_readStatus = time.time()
			# --- reg_driverOutputCommand --- #                            
			# try:
			# 	time.sleep(self.timeWait)
			# 	rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.READ_HOLDING_REGISTERS, self.reg_driverOutputCommand_lower, 1)
			# 	arrData = bin(rawData[0])
			# 	l = len(arrData) - 2
			# 	for i in range(16 - l):
			# 		allData += '0'
			# 	for i in range(l):
			# 		allData += arrData[i + 2]
			
			# 	#self.statusDriver_1.FWD = int(allData[12])                
			# 	#self.statusDriver_1.REV = int(allData[11])

			# 	self.statusDriver_1.STOP_MODE = int(allData[10]) # - 5
			# 	# self.statusDriver_1.WNG = int(allData[9])                 

			# 	self.statusDriver_1.ALARM_OUT1 = int(allData[8]) # - 7 [ALM-A]
			# 	self.statusDriver_1.S_BSY = int(allData[7]) # - 8

			# 	# self.statusDriver_1.ALARM_OUT2 = int(allData[3])         

			# 	self.statusDriver_1.MOVE = int(allData[2]) # - 13
			# 	self.statusDriver_1.VA = int(allData[1]) # - 14
			# 	self.statusDriver_1.TLC = int(allData[0]) # - 15

			# except modbus_tk.modbus.ModbusError as exc:
			# 	print ("Read reg_driverOutputCommand_lower ID1 Error")
			# 	self.countErr_reqSpeed += 1

			# --- PRESENT_ALARM --- #
			time.sleep(self.timeWait)
			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.READ_HOLDING_REGISTERS, self.reg_presentAlarm, 1)
				self.statusDriver_1.PRESENT_ALARM = rawData[0]
			except modbus_tk.modbus.ModbusError as exc:
				print ("Read reg_presentAlarm ID1 Error")
				self.countErr_reqSpeed += 1

			# --- PRESENT_WARNING --- #
			# if (self.statusDriver_1.WNG != 0):
			# 	try:
			# 		time.sleep(self.timeWait)
			# 		rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.READ_HOLDING_REGISTERS, self.reg_presentWarning, 1)
			# 		self.statusDriver_1.PRESENT_WARNING = rawData[0]
			# 	except modbus_tk.modbus.ModbusError as exc:
			# 		print ("Read reg_presentWarning ID1 Error")
			# 		self.countErr_reqSpeed += 1
			# else:
			# 	self.statusDriver_1.PRESENT_WARNING = 0
				
		# ----------------- Driver 2 ----------------- #
			# --- reg_driverOutputCommand --- #
			# try:
			# 	time.sleep(self.timeWait)
			# 	rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.READ_HOLDING_REGISTERS, self.reg_driverOutputCommand_lower, 1)
			# 	arrData = bin(rawData[0])
			# 	l = len(arrData) - 2
			# 	for i in range(16 - l):
			# 		allData += '0'
			# 	for i in range(l):
			# 		allData += arrData[i + 2]
			
			# 	# self.statusDriver_2.FWD = int(allData[12])
			# 	# self.statusDriver_2.REV = int(allData[11])

			# 	self.statusDriver_2.STOP_MODE = int(allData[10]) # - 5
			# 	# self.statusDriver_2.WNG = int(allData[9])

			# 	self.statusDriver_2.ALARM_OUT1 = int(allData[8]) # - 7 [ALM-A]
			# 	self.statusDriver_2.S_BSY = int(allData[7]) # - 8

			# 	# self.statusDriver_2.ALARM_OUT2 = int(allData[3])

			# 	self.statusDriver_2.MOVE = int(allData[2]) # - 13
			# 	self.statusDriver_2.VA = int(allData[1]) # - 14
			# 	self.statusDriver_2.TLC = int(allData[0]) # - 15

			# except modbus_tk.modbus.ModbusError as exc:
			# 	print ("Read reg_driverOutputCommand_lower ID2 Error")	
			# 	self.countErr_reqSpeed += 1

			# --- PRESENT_ALARM --- #
			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.READ_HOLDING_REGISTERS, self.reg_presentAlarm, 1)
				self.statusDriver_2.PRESENT_ALARM = rawData[0]
			except modbus_tk.modbus.ModbusError as exc:
				print ("Read reg_presentAlarm ID2 Error")
				self.countErr_reqSpeed += 1

			# --- PRESENT_WARNING --- #
			# if (self.statusDriver_2.WNG != 0):
			# 	try:
			# 		time.sleep(self.timeWait)
			# 		rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.READ_HOLDING_REGISTERS, self.reg_presentWarning, 1)
			# 		self.statusDriver_2.PRESENT_WARNING = rawData[0]
			# 	except modbus_tk.modbus.ModbusError as exc:
			# 		print ("Read reg_presentWarning ID2 Error")
			# 		self.countErr_reqSpeed += 1
			# else:
			# 	self.statusDriver_2.PRESENT_WARNING = 0

# ------------------------------------------------------
	def spin(self, ctrlDriver): # controlDriver()
		""" 
			MB-FREE: The electromagnetic brake.
			STOPMODE: 0: Instantaneous stop | 1: Deceleration stop.
		NB	MB-FREE | Not used | STOPMODE | REV | FWD | M2 | M1 | M0 
		178     1   |    0     |     1    |  1  |  0  |  0 |  0 |  0
		170	    1   |    0     |     1    |  0  |  1  |  0 |  0 |  0
		32	    0   |    0     |     1    |  0  |  0  |  0 |  0 |  0
		0	    0   |    0     |     0    |  0  |  0  |  0 |  0 |  0
		"""
		if (ctrlDriver.SPEED == 0):
			# -- STOP MODE
			if (ctrlDriver.STOP_MODE == 1): # - Instantaneous stop.
				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(ctrlDriver.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 0)
					# print ("rawData1: ", rawData)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_inputCommandLower Error")
					self.countErr_reqSpeed += 1

				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(ctrlDriver.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 0)
					# print ("rawData1: ", rawData)
					# print ("here")
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_inputCommandLower Error")
					self.countErr_reqSpeed += 1

			else: # - Deceleration stop.
				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(ctrlDriver.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 0)
					# print ("rawData1: ", rawData)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_inputCommandLower Error")
					self.countErr_reqSpeed += 1

				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(ctrlDriver.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 0)
					# print ("rawData1: ", rawData)
					# print ("here")
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_inputCommandLower Error")
					self.countErr_reqSpeed += 1

		else:
			# -- Turn on S-ON
			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(ctrlDriver.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 1)
				# print ("rawData1: ", rawData)
			except modbus_tk.modbus.ModbusError as exc:
				print ("Read reg_inputCommandLower Error")
				self.countErr_reqSpeed += 1

			# -- Operation Data Number
			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(self.ID_Driver, cst.WRITE_SINGLE_REGISTER, self.reg_operationData, output_value = 2)
				# print ("reg 89: ", rawData)
			except modbus_tk.modbus.ModbusError as exc:
				print ("reg_operationData Error")
				self.countErr_reqSpeed += 1

			# -- Operating No.2 - Type
			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(ctrlDriver.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_type, output_value = 16) #
				# print ("reg 6145: ", rawData)
			except modbus_tk.modbus.ModbusError as exc:
				print ("reg_operationDataNo2_type Error")
				self.countErr_reqSpeed += 1
				
			# -- DIRECTION
			if (ctrlDriver.SPEED > 0):
				# -- REVERT
				if (ctrlDriver.REVERT == 0):
					try:
						time.sleep(self.timeWait)
						rawData = self.MODBUS.execute(ctrlDriver.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 516) # 
						# print ("reg 8: ", rawData)
					except modbus_tk.modbus.ModbusError as exc:
						print ("reg_inputCommandUpper Error")
						self.countErr_reqSpeed += 1
				else:
					try:
						time.sleep(self.timeWait)
						rawData = self.MODBUS.execute(ctrlDriver.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 520) # 
						# print ("reg 8: ", rawData)
					except modbus_tk.modbus.ModbusError as exc:
						print ("reg_inputCommandUpper Error")
						self.countErr_reqSpeed += 1
			else:
				# -- REVERT
				if (ctrlDriver.REVERT == 0):
					try:
						time.sleep(self.timeWait)
						rawData = self.MODBUS.execute(ctrlDriver.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 520) # 
						# print ("reg 8: ", rawData)
					except modbus_tk.modbus.ModbusError as exc:
						print ("reg_inputCommandUpper Error")
						self.countErr_reqSpeed += 1
				else:
					try:
						time.sleep(self.timeWait)
						rawData = self.MODBUS.execute(ctrlDriver.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 516) # 
						# print ("reg 8: ", rawData)
					except modbus_tk.modbus.ModbusError as exc:
						print ("reg_inputCommandUpper Error")
						self.countErr_reqSpeed += 1

		# -- SPEED
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(ctrlDriver.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_velocity, output_value = abs(ctrlDriver.SPEED) )
			# print ("reg 6149: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_operationDataNo2_velocity Error")
			self.countErr_reqSpeed += 1

	def publish_status(self): # - OK
		t_pub = time.time() - self.preTime_pubStatus
		if (t_pub >= self.cycle_pubStatus):
			self.preTime_pubStatus = time.time()
			self.pub_driverRespond.publish(self.driver_respond)

	def config_All(self): # - OK
		# -- Time - Communicate Timeout
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_communicationTimeout, output_value = 500 ) # ms
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_communicationTimeout ID1 Error")
			self.countErr_reqSpeed += 1

		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_communicationTimeout, output_value = 500 ) # ms
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_communicationTimeout ID2 Error")
			self.countErr_reqSpeed += 1

		# -- Counter - Communicate Timeout to warning
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_CommunicationErrorDetection, output_value = 3 ) # times
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_CommunicationErrorDetection ID1 Error")	
			self.countErr_reqSpeed += 1

		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_CommunicationErrorDetection, output_value = 3 ) # times
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_CommunicationErrorDetection ID2 Error")	
			self.countErr_reqSpeed += 1

		# -- Operation Data Number - Select No.2
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationData, output_value = 2)
			# print ("reg 89: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_operationData ID1 Error")
			self.countErr_reqSpeed += 1

		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationData, output_value = 2)
			# print ("reg 89: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_operationData ID2 Error")
			self.countErr_reqSpeed += 1

		# -- Operating No.2 - Type
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_type, output_value = 16) #
			# print ("reg 6145: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_operationDataNo2_type ID1 Error")
			self.countErr_reqSpeed += 1

		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_type, output_value = 16) #
			# print ("reg 6145: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_operationDataNo2_type ID2 Error")
			self.countErr_reqSpeed += 1

		# -- Operating No.2 - Acceleration Time
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_accelerationTime, output_value = self.accelerationRate) # 
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_operationDataNo2_accelerationTime ID1 Error")
			self.countErr_reqSpeed += 1

		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_accelerationTime, output_value = self.accelerationRate) # 
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_operationDataNo2_accelerationTime ID2 Error")
			self.countErr_reqSpeed += 1

		# -- Operating No.2 - Deceleration Time
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_decelerationTime, output_value = self.decelerationRate) # 
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_operationDataNo2_decelerationTime ID1 Error")
			self.countErr_reqSpeed += 1

		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_decelerationTime, output_value = self.decelerationRate) # 
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_operationDataNo2_decelerationTime ID2 Error")
			self.countErr_reqSpeed += 1

		# -- Operating No.2 - Torque Limiting
		# try:
		# 	time.sleep(self.timeWait)
		# 	rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_torqueLimiting, output_value = self.torqueLimiting ) # 
		# except modbus_tk.modbus.ModbusError as exc:
		# 	print ("reg_operationDataNo2_torqueLimiting ID1 Error")
		# 	self.countErr_reqSpeed += 1	

		# try:
		# 	time.sleep(self.timeWait)
		# 	rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_torqueLimiting, output_value = self.torqueLimiting ) # 
		# except modbus_tk.modbus.ModbusError as exc:
		# 	print ("reg_operationDataNo2_torqueLimiting ID2 Error")
			self.countErr_reqSpeed += 1	

		# -- Operating No.2 - S-ON
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 0) # 
			# print ("reg 8: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_inputCommandUpper ID1 Error")
			self.countErr_reqSpeed += 1

		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 0) # 
			# print ("reg 8: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_inputCommandUpper ID2 Error")
			self.countErr_reqSpeed += 1

		# -- Operating No.2 - DIRECTION
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 0) #
			# print ("reg 8: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_inputCommandLower ID1 Error")
			self.countErr_reqSpeed += 1

		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 0) #
			# print ("reg 8: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_inputCommandLower ID2 Error")
			self.countErr_reqSpeed += 1

	def check_query(self): # - OK
		if (self.is_readQuery_1 == 1 or self.is_readQuery_2 == 1):
			t = time.time() - self.lastTime_checkQuery_1
			if (t >= self.cycle_checkQuery):
				return 0
			else:
				return 1
		else:
			return 0

	def spin_all(self, spd_1, spd_2, stopMode_1, stopMode_2):
		vel_1 = spd_1
		vel_2 = spd_2
		""" 
			MB-FREE: The electromagnetic brake.
			STOPMODE: 0: Instantaneous stop | 1: Deceleration stop.
		NB	MB-FREE | Not used | STOPMODE | REV | FWD | M2 | M1 | M0 
		178     1   |    0     |     1    |  1  |  0  |  0 |  0 |  0
		170	    1   |    0     |     1    |  0  |  1  |  0 |  0 |  0
		32	    0   |    0     |     1    |  0  |  0  |  0 |  0 |  0
		0	    0   |    0     |     0    |  0  |  0  |  0 |  0 |  0
		"""
		# -- DIRECTION
		rev_1 = self.controlDriver_1.REVERT
		rev_2 = self.controlDriver_2.REVERT
		# ---------------------------  Driver 1 --------------------------- #
		if (spd_1 == 0):				
			# pass
			if (self.disable_brake.data == 0):
				# -- S-ON
				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 1)
					# print ("rawData1: ", rawData)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_inputCommandLower Error")
					self.countErr_reqSpeed += 1
			else:
				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 65) # 65
					# print ("rawData1: ", rawData)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_inputCommandLower Error")
					self.countErr_reqSpeed += 1

			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 0) # 
				# print ("reg 8: ", rawData)
			except modbus_tk.modbus.ModbusError as exc:
				print ("reg_inputCommandUpper Error")
				self.countErr_reqSpeed += 1

		else:
			# ----- Setup ----- #
			# -- Turn on S-ON
			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 1)
				# print ("rawData1: ", rawData)
			except modbus_tk.modbus.ModbusError as exc:
				print ("Read reg_inputCommandLower Error")
				self.countErr_reqSpeed += 1

			# -- Operation Data Number
			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationData, output_value = 2)
				# print ("reg 89: ", rawData)
			except modbus_tk.modbus.ModbusError as exc:
				print ("reg_operationData Error")
				self.countErr_reqSpeed += 1

			# -- Operating No.2 - Type
			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_type, output_value = 16) #
				# print ("reg 6145: ", rawData)
			except modbus_tk.modbus.ModbusError as exc:
				print ("reg_operationDataNo2_type Error")
				self.countErr_reqSpeed += 1

			# ----- Direction ----- #
			if (spd_1 > 0):
				# -- REVERT
				if (rev_1 == 0):
					try:
						time.sleep(self.timeWait)
						rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 516) # 
						# print ("reg 8: ", rawData)
					except modbus_tk.modbus.ModbusError as exc:
						print ("reg_inputCommandUpper Error")
						self.countErr_reqSpeed += 1
				else:
					try:
						time.sleep(self.timeWait)
						rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 520) # 
						# print ("reg 8: ", rawData)
					except modbus_tk.modbus.ModbusError as exc:
						print ("reg_inputCommandUpper Error")
						self.countErr_reqSpeed += 1
			else:
				# -- REVERT
				if (rev_1 == 0):
					try:
						time.sleep(self.timeWait)
						rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 520) # 
						# print ("reg 8: ", rawData)
					except modbus_tk.modbus.ModbusError as exc:
						print ("reg_inputCommandUpper Error")
						self.countErr_reqSpeed += 1
				else:
					try:
						time.sleep(self.timeWait)
						rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 516) # 
						# print ("reg 8: ", rawData)
					except modbus_tk.modbus.ModbusError as exc:
						print ("reg_inputCommandUpper Error")
						self.countErr_reqSpeed += 1

		# ---------------- Setup --------------- #
		# -- Driver 2.
		if (spd_2 == 0):
			if (self.disable_brake.data == 0):
				# -- S-ON
				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 1)
					# print ("rawData1: ", rawData)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_inputCommandLower Error")
					self.countErr_reqSpeed += 1
			else:
				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 65) # 65
					# print ("rawData1: ", rawData)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_inputCommandLower Error")
					self.countErr_reqSpeed += 1

			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 0) # 
				# print ("reg 8: ", rawData)
			except modbus_tk.modbus.ModbusError as exc:
				print ("reg_inputCommandUpper Error")
				self.countErr_reqSpeed += 1
				
		else:
			# ----- Setup ----- #
			# -- Turn on S-ON
			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 1)
				# print ("rawData1: ", rawData)
			except modbus_tk.modbus.ModbusError as exc:
				print ("Read reg_inputCommandLower Error")
				self.countErr_reqSpeed += 1

			# -- Operation Data Number
			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationData, output_value = 2)
				# print ("reg 89: ", rawData)
			except modbus_tk.modbus.ModbusError as exc:
				print ("reg_operationData Error")
				self.countErr_reqSpeed += 1

			# -- Operating No.2 - Type
			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_type, output_value = 16) #
				# print ("reg 6145: ", rawData)
			except modbus_tk.modbus.ModbusError as exc:
				print ("reg_operationDataNo2_type Error")
				self.countErr_reqSpeed += 1

			# ----- Direction ----- #
			if (spd_2 > 0):
				# -- REVERT
				if (rev_2 == 0):
					try:
						time.sleep(self.timeWait)
						rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 516) # 
						# print ("reg 8: ", rawData)
					except modbus_tk.modbus.ModbusError as exc:
						print ("reg_inputCommandUpper Error")
						self.countErr_reqSpeed += 1
				else:
					try:
						time.sleep(self.timeWait)
						rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 520) # 
						# print ("reg 8: ", rawData)
					except modbus_tk.modbus.ModbusError as exc:
						print ("reg_inputCommandUpper Error")
						self.countErr_reqSpeed += 1
			else:
				# -- REVERT
				if (rev_2 == 0):
					try:
						time.sleep(self.timeWait)
						rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 520) # 
						# print ("reg 8: ", rawData)
					except modbus_tk.modbus.ModbusError as exc:
						print ("reg_inputCommandUpper Error")
						self.countErr_reqSpeed += 1
				else:
					try:
						time.sleep(self.timeWait)
						rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandUpper, output_value = 516) # 
						# print ("reg 8: ", rawData)
					except modbus_tk.modbus.ModbusError as exc:
						print ("reg_inputCommandUpper Error")
						self.countErr_reqSpeed += 1
		# ---------------- Speed --------------- #
		# ----- Speed 1 ----- #
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_velocity, output_value = abs(spd_1) )
			# print ("reg 6149: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_operationDataNo2_velocity ID_1 Error")
			self.countErr_reqSpeed += 1

		# ----- Speed 2 ----- #
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_operationDataNo2_velocity, output_value = abs(spd_2) )
			# print ("reg 6149: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("reg_operationDataNo2_velocity ID_2 Error")
			self.countErr_reqSpeed += 1

	def run(self):
		self.config_All()

		self.getStatus_all(1)

		self.resetAll()

		self.getStatus_all(1)

		# -- Turn on S-ON
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_1.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 1)
			# print ("rawData1: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_inputCommandLower Error")
			self.countErr_reqSpeed += 1

		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.controlDriver_2.ID, cst.WRITE_SINGLE_REGISTER, self.reg_inputCommandLower, output_value = 1)
			# print ("rawData1: ", rawData)
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_inputCommandLower Error")
			self.countErr_reqSpeed += 1

		while not rospy.is_shutdown():
			# -- check communicate
			if (self.check_query() == 1): #  and  self.check_safety() == 1
				enable_run = 1
				self.driverRespond_1.status = self.status_run
				self.driverRespond_2.status = self.status_run
				self.value_runing = 1
			else:
				enable_run = 0
				self.driverRespond_1.status = self.status_stop
				self.driverRespond_2.status = self.status_stop
				self.value_runing = 0
			# ----------------------------------- #
			# -- Task Reset
			if (self.driverQuery_1.task == 1):
				self.resetAll()

			# -- Task Read Status
			self.getStatus_all(0)
			self.getSpeed_all()
			# self.getPosition_all()

			# -- Control speed
			if (enable_run):
				self.spin_all(self.driverQuery_1.rotationSpeed, self.driverQuery_2.rotationSpeed, self.driverQuery_1.modeStop, self.driverQuery_2.modeStop)
			else:
				# print ("here!")
				self.spin_all(0, 0, 1, 1)
			# ----------------------------------- #
			# -- driver 1
			self.driverRespond_1.REV = self.statusDriver_1.REV
			self.driverRespond_1.FWD = self.statusDriver_1.FWD
			self.driverRespond_1.speed = self.statusDriver_1.SPEED
			self.driverRespond_1.alarm_all = self.statusDriver_1.PRESENT_ALARM
			# self.driverRespond_1.alarm_overload = self.statusDriver_1.ALARM_OUT1
			self.driverRespond_1.warning = self.statusDriver_1.ALARM_OUT1
			self.driverRespond_1.message_error = self.mean_ALM_A(self.statusDriver_1.PRESENT_ALARM)
			self.driverRespond_1.message_warning = self.mean_ALM_A(self.statusDriver_1.PRESENT_ALARM)
			# -- driver 2
			self.driverRespond_2.REV = self.statusDriver_2.REV
			self.driverRespond_2.FWD = self.statusDriver_2.FWD
			self.driverRespond_2.speed = self.statusDriver_2.SPEED
			self.driverRespond_2.alarm_all = self.statusDriver_2.PRESENT_ALARM
			# self.driverRespond_2.alarm_overload = self.statusDriver_2.ALARM_OUT1
			self.driverRespond_2.warning = self.statusDriver_2.ALARM_OUT1
			self.driverRespond_2.message_error = self.mean_ALM_A(self.statusDriver_2.PRESENT_ALARM)
			self.driverRespond_2.message_warning = self.mean_ALM_A(self.statusDriver_2.PRESENT_ALARM)
			# ----------------------------------- #
			self.pub_driverRespond_1.publish(self.driverRespond_1)
			self.pub_driverRespond_2.publish(self.driverRespond_2)

			self.rate.sleep()

		print('Programer stopped')


def main():
	print('Starting main program')

	program = driver()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()

"""
Input signals list
"""
