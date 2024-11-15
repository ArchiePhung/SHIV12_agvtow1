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
		self.ID_Driver = rospy.get_param("id", 1)

		self.maxRPM = rospy.get_param("maxRPM", 3800)
		self.minSpeed = self.maxRPM*(-1)
		self.topicSafety = rospy.get_param("topicSafety", "/safety")
		self.topicSub = rospy.get_param("topicSub", "/driver1_query")
		self.topicPub = rospy.get_param("topicPub", "/driver1_respond")
		self.revert = rospy.get_param("revert", 0)

		self.lastTime_readStatus = time.time()
		self.time_readStatus = 2 # s
		# -- 
		self.perimeter = 0.47
		self.transmission_ratio = 30

		self.accelerationTime = 2 # n*0.1s - 10
		self.decelerationTime = 2 # n*0.1s
		self.torqueLimiting = 200
		# -- Registers
		self.reg_driverInputCommand = 125 # 0x7F - pape 19 : điều khiển chiều, chế độ.
		self.reg_driverOutputCommand = 127 # 0x7F - pape 20 : Đọc trạng thái driver, tốc độ, ....
		self.reg_resetAlarm = 385 # 0x181 
		self.reg_clearAlarmRecords = 389 # 0x185
		self.reg_clearWarningRecords = 391 # 0x178
		self.reg_clearCommunicationError = 393 # 0x189
		self.reg_configuration = 397 # 0x18D
		self.reg_allDataIntialization = 399 # 0x18F
		self.reg_batchNvMemoryRead = 401 # 0x191
		self.reg_batchNvMemoryWrite = 403 # 0x193 

		self.reg_rotationSpeed_No2 = 1157 # 0x485 - follow mode 0 - default. Operation data setting using analog input signal selection - See pape 28
		self.reg_acceleration_No2 = 1541  # 0x605 - follow mode 0 - default. 
		self.reg_deceleration_No2 = 1669  # 0x685 - follow mode 0 - default. 
		self.reg_torqueLimiting_No2 = 1797  # 0x705 - follow mode 0 - default. 
		# -- See pape 29
		self.reg_underVoltageWarningLevel = 841  # 0x349 - 0.1 * n | n [0, 480]. 
		self.reg_brakeActionAtAlarm = 4225  # 
		self.reg_underVoltageAlarmLatch = 4229  # 0x1085 - cut off motor when releasing the undervoltage alarm. 
		self.reg_overloadWarningFunction = 4259  # 0x10A3 [0, 1] - disable/enable warning . 
		self.reg_UndervoltageWarningFunction = 4265  # 0x10A9 - [0, 1] - disable/enable warning. 
		self.reg_overloadWarningLevel = 4267  # 0x10AB - [0, 100]. 
		self.reg_communicationTimeout = 4609  # 0x1201 - [0, 10000] 1 ms.
		self.reg_communicationErrorAlarm = 4611  # 0x1203 - [0, 10] count numbers timeout to warning - 3 default. 
		# -- Monitor commands pape 23.
		self.reg_presentAlarm = 129 # 0x81
		self.reg_presentWarning = 151 # 0x97
		self.reg_communicationErrorCode = 173 # 0xAD
		self.reg_presentSelectedDataNo = 197 # 0xC5
		self.reg_commandSpeed = 201 # 0xC9
		self.reg_feedbackSpeed = 207 # 0xCF

		# -- 
		self.statusDriver = statusDriver(self.ID_Driver)
		self.controlDriver = controlDriver(self.ID_Driver, self.revert)


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
		self.rate = rospy.Rate(1000)

		rospy.Subscriber(self.topicSafety, Bool, self.safety_callback)
		self.safety = Bool()
		self.enable_run = 0
		self.is_readSafety = 0
		self.lastTime_checkSafety = time.time()
		self.cycle_checkSafety = 1 # s

		rospy.Subscriber(self.topicSub, Driver_query, self.driverQuery_callback)
		self.driver_query = Driver_query()
		self.is_readQuery = 0
		self.lastTime_checkQuery = time.time()
		self.cycle_checkQuery = 0.5 # s

		self.pub_driverRespond = rospy.Publisher(self.topicPub, Driver_respond, queue_size=50)
		self.driver_respond = Driver_respond()

		self.status_error = 2
		self.status_run = 1
		self.status_stop = 0
		# -- 
		self.timeWait = 0.001
		# -- 
		self.frequence_pubStatus = 25.
		self.cycle_pubStatus = 1/self.frequence_pubStatus
		self.preTime_pubStatus = time.time()
		# --
		self.countErr_reqSpeed = 0

	def driverQuery_callback(self, data):
		self.driver_query = data
		self.is_readQuery = 1
		self.lastTime_checkQuery = time.time()

	def safety_callback(self, data):
		self.safety = data
		self.is_readSafety = 1
		self.lastTime_checkSafety = time.time()

	def errorType_All(self, err):
		mess = ' '
		switcher={
			0:   'All right!',
			132: 'ERR 84: RS-485 communication error',
			136: 'ERR 88: Command not yet defined',
			137: 'ERR 89: User I/F communication in progress',
			138: 'ERR 8A: NV memory processing in progress',
			140: 'ERR 8C: Outside setting range',
			141: 'ERR 8D: Command execute disable',
			33:  'WAR 21: Main circuit overheat',
			37:  'WAR 25: Undervoltage∗',
			48:  'WAR 30: Overload∗',
			108: 'WAR 6C: Operation error '

		}
		return switcher.get(err, 'Not found!')
	
	def casuse_all(self, cas):
		mess = ' '
		switcher={
			0:   'All right!',
			132: 'ERR 84h: One of the following errors was detected: Framing error or BCC error',
			136: 'ERR 88h: The command requested by the master could not be executed because of being undefined | An exception response (exception code 01h, 02h) was detected. See p.15',
			137: 'ERR 89h: The command requested by the master could not be executed since the OPX-2A was communicating with the driver',
			138: 'ERR 8Ah: The command could not be executed because the driver was processing the NV memory. Internal processing was in progress. (S-BSY is ON.) An EEPROM error alarm was present | An exception response (exception code 04h) was detected. See p.15',
			140: 'ERR 8C: The setting data requested by the master could not be executed due to outside the range | An exception response (exception code 03h, 04h) was detected. See p.15',
			141: 'ERR 8D: When the command could not be executed, it tried to do it.| An exception response (exception code 04h) was detected. See p.15',
			33:  'WAR 21: The temperature inside the driver exceeded the overheat warning level',
			37:  'WAR 25:The main power supply voltage dropped by approx. 10% or more from the rated voltage. ',
			48:  'WAR 30: The load torque of the motor exceeded the overload warning level.',
			108: 'WAR 6C: When performing test operation or changing the assignment of the input terminal using the OPX-2A '			
		}
		return switcher.get(cas, 'Not found!')

	def methor_all(self, cas):
		mess = ' '
		switcher={
			0:   'All right!',
			132: 'ERR 84h: Check the connection, programmable controller, setting of RS-485 communication',
			136: 'ERR 88h: Check the setting value for the command | Check the flame configuration',
			137: 'ERR 89h: Wait until the processing for the OPX-2A will be completed',
			138: 'ERR 8Ah: Wait until the internal processing will complete | • When the EEPROM error was generated, initialize the parameter using OPX-2A or RS-485 communication',
			140: 'ERR 8C: Check the setting data',
			141: 'ERR 8D: Check the driver status',
			33:  'WAR 21: The temperature inside the driver exceeded the overheat warning level',
			37:  'WAR 25:The main power supply voltage dropped by approx. 10% or more from the rated voltage. ',
			48:  'WAR 30: The load torque of the motor exceeded the overload warning level.',
			108: 'WAR 6C: When performing test operation or changing the assignment of the input terminal using the OPX-2A '			
		}
		return switcher.get(cas, 'Not found!')

	def errorType_Alarm(self, err):
		# Reset using the ALARMRESET input: <
		mess = ' '
		switcher={
			0:   'All right!',
			48: '30 < Blinks:2 Overload',
			40: '28 < Blinks:3 Sensor error',
			66: '42 < Blinks:3 Initial sensor error',
			34: '22 < Blinks:4 Overvoltage',
			37: '25 < Blinks:5 Undervoltage',
			49: '31 < Blinks:6 Overspeed',
			32: '20 Blinks:7 Overcurrent',
			65: '41 Blinks:8 EEPROM error',
			33: '21 < Blinks:9 Main circuit overheat',
			110: '6E < Blinks:10 External stop∗1',
			70:  '46 < Blinks:11 Initial operation error∗2',
			129: '81 < Blinks:12 Network bus error',
			131: '83 Blinks:12 Communication switch setting error',
			132: '84 < Blinks:12 RS-485 communication error',
			133: '85 < Blinks:12 RS-485 communication timeout',
			142: '8E < Blinks:12 Network converter error ',
			45:  '2D < Blinks:14 Main circuit output error∗3'
		}
		return switcher.get(err, 'Not found!')
	
	def casuse_Alarm(self, cas):
		# Reset using the ALARMRESET input: <
		mess = ' '
		switcher={
			0:  'All right!',
			48: '30 < A load exceeding the rated torque was applied to the motor for 5 seconds or more',
			40: '28 < The motor sensor signal line experienced an open circuit during operation, or the motor signal connector came off',
			66: '42 < The motor sensor signal line broke or motor signal connector came off before the main power supply was turned on',
			34: '22 < The main power supply voltage exceeded the overvoltage detection level [Detection level] BLV620: approx. 40 VDC or BLV640: approx. 72 VDC | Sudden starting/stopping of a large inertia load was performed',
			37: '25 < The main power supply voltage dropped the undervoltage detection level [Detection level] BLV620: approx 10 VDC or BLV640: approx 20 VDC',
			49: '31 < The rotation speed of the motor output shaft exceeded approx 4800 r/min',
			32: '20 Excessive current has flown through the driver due to ground fault, etc',
			65: '41 Stored data was damaged. Data became no longer writable or readable',
			33: '21 < The temperature inside the driver exceeded the main circuit overheat level',
			110: '6E < The EXT-ERROR input turned OFF',
			70:  '46 < The main power supply was cycled when the FWD input or REV input was ON',
			129: '81 < The bus of host network of the network converter turned off while the motor was operating',
			131: '83 The communication function switch (SW2-No.4) was turned ON',
			132: '84 < The number of consecutive RS-485 communication errors reached the value set in the "communication error alarm" parameter',
			133: '85 < The time set in the “communication timeout” parameter has elapsed, and yet the communication could not be established with the host system',
			142: '8E < The network converter generated an alarm',
			45:  '2D < The motor drive wire broke or motor drive connector came off'			
		}
		return switcher.get(cas, 'Not found!')

	def methor_Alarm(self, cas):
		mess = ' '
		switcher={
			0:   'All right!',
			48: '30 Decrease the load. Review the operation pattern such as acceleration/deceleration time',
			40: '28 Check the connection between the driver and motor',
			66: '42 Check the connection between the driver and motor',
			34: '22 Check power supply | ... Pape 36 manual Rs485',
			37: '25 Check the main power supply voltage | Check the wiring of the power supply cable',
			49: '31 Decrease the load | Review the operation pattern such as acceleration/ deceleration time',
			32: '20 Check the wiring between the driver and motor for damage and cycle the power',
			65: '41 Initialize the parameters using the OPX-2A, and cycle the power',
			33: '21 Review the ventilation condition in the enclosure',
			110: '6E Check the EXT-ERROR input',
			70:  '46 Turn the FWD input and REV input OFF, and then cycle the main power supply',
			129: '81 Check the connector and cable of the host network',
			131: '83 Check the communication function switch (SW2-No.4)',
			132: '84 • Check the connection with the host system | Check the setting of the RS-485 communication',
			133: '85 Check the connection with the host system',
			142: '8E Check the alarm code of the network converter',
			45: '2D Check the connection between the driver and motor'
		}
		return switcher.get(cas, 'Not found!')

	def getStatus_now(self):
		rawData = 0
		allData = ''
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.ID_Driver, cst.READ_HOLDING_REGISTERS, self.reg_driverOutputCommand, 1)
			arrData = bin(rawData[0])
			l = len(arrData) - 2
			for i in range(16 - l):
				allData += '0'
			for i in range(l):
				allData += arrData[i + 2]
			# print "len: ", len(allData)
			# print "allData: ", allData
		
			self.statusDriver.FWD = int(allData[12])
			self.statusDriver.REV = int(allData[11])
			self.statusDriver.STOP_MODE = int(allData[10])
			self.statusDriver.WNG = int(allData[9])
			self.statusDriver.ALARM_OUT1 = int(allData[8])
			self.statusDriver.S_BSY = int(allData[7])
			self.statusDriver.ALARM_OUT2 = int(allData[3])
			self.statusDriver.MOVE = int(allData[2])
			self.statusDriver.VA = int(allData[1])
			self.statusDriver.TLC = int(allData[0])

		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_driverOutputCommand error")
			sys.exit()

		if (self.statusDriver.ALARM_OUT1 != 0):
			time.sleep(self.timeWait)
			try:
				rawData = self.MODBUS.execute(self.ID_Driver, cst.READ_HOLDING_REGISTERS, self.reg_presentAlarm, 1)
				self.statusDriver.PRESENT_ALARM = rawData[0]
			except modbus_tk.modbus.ModbusError as exc:
				print ("Read reg_presentAlarm error")
				sys.exit()
										
		else:
			self.statusDriver.PRESENT_ALARM = 0

		if (self.statusDriver.WNG != 0):
			time.sleep(self.timeWait)
			try:
				rawData = self.MODBUS.execute(self.ID_Driver, cst.READ_HOLDING_REGISTERS, self.reg_presentWarning, 1)
				self.statusDriver.PRESENT_WARNING = rawData[0]
			except modbus_tk.modbus.ModbusError as exc:
				print ("Read reg_presentWarning error")
				sys.exit()
		else:
			self.statusDriver.PRESENT_WARNING = 0
		# --------------------------------- SPEED
		self.getSpeed()	

	def getStatus(self):
		# -- get vel, alarm, warning, error, ...
		rawData = 0
		allData = ''
		# ---------------------------------
		
		t = (time.time() - self.lastTime_readStatus)%60 
		if (t > self.time_readStatus):
			self.lastTime_readStatus = time.time()
			try:
				time.sleep(self.timeWait)
				rawData = self.MODBUS.execute(self.ID_Driver, cst.READ_HOLDING_REGISTERS, self.reg_driverOutputCommand, 1)
				arrData = bin(rawData[0])
				l = len(arrData) - 2
				for i in range(16 - l):
					allData += '0'
				for i in range(l):
					allData += arrData[i + 2]
				# print "len: ", len(allData)
				# print "allData: ", allData
			
				self.statusDriver.FWD = int(allData[12])
				self.statusDriver.REV = int(allData[11])
				self.statusDriver.STOP_MODE = int(allData[10])
				self.statusDriver.WNG = int(allData[9])
				self.statusDriver.ALARM_OUT1 = int(allData[8])
				self.statusDriver.S_BSY = int(allData[7])
				self.statusDriver.ALARM_OUT2 = int(allData[3])
				self.statusDriver.MOVE = int(allData[2])
				self.statusDriver.VA = int(allData[1])
				self.statusDriver.TLC = int(allData[0])

			except modbus_tk.modbus.ModbusError as exc:
				print ("Read reg_driverOutputCommand error")
				sys.exit()

			if (self.statusDriver.ALARM_OUT1 != 0):
				time.sleep(self.timeWait)
				try:
					rawData = self.MODBUS.execute(self.ID_Driver, cst.READ_HOLDING_REGISTERS, self.reg_presentAlarm, 1)
					self.statusDriver.PRESENT_ALARM = rawData[0]
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_presentAlarm error")
					sys.exit()
											
			else:
				self.statusDriver.PRESENT_ALARM = 0

			if (self.statusDriver.WNG != 0):
				time.sleep(self.timeWait)
				try:
					rawData = self.MODBUS.execute(self.ID_Driver, cst.READ_HOLDING_REGISTERS, self.reg_presentWarning, 1)
					self.statusDriver.PRESENT_WARNING = rawData[0]
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_presentWarning error")
					sys.exit()
			else:
				self.statusDriver.PRESENT_WARNING = 0
		# --------------------------------- SPEED
		self.getSpeed()

		# self.statusDriver.showAll()

	def getSpeed(self):
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(self.ID_Driver, cst.READ_HOLDING_REGISTERS, self.reg_feedbackSpeed, 1)
			if (rawData[0] > 6000):
				spd = rawData[0] - pow(2, 16)
			else:
				spd = rawData[0]

			if (self.revert == 1):
				self.statusDriver.SPEED = -spd
			else:
				self.statusDriver.SPEED = spd

		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_feedbackSpeed error")
			sys.exit()

	def stopInstantaneous(self, id):
		rawData = 0
		# ---------------------------------
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 0)
			# print ("rawData: ", rawData
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_driverInputCommand error")

	def stopDeceleration(self, id):
		rawData = 0
		# ---------------------------------
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 32)
			# print ("rawData: ", rawData
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_driverInputCommand error")

	def resetAlarm(self, id):
		rawData = 0
		# ---------------------------------
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_resetAlarm, output_value = 1)
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read resetAlarm error")
			sys.exit()
		# ---------------------------------
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_resetAlarm, output_value = 0)
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read resetAlarm error")
			sys.exit()

	def spin(self, stopMode, spd):
		vel = spd
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
		rev = self.controlDriver.REVERT

		if (spd > 0):
			vel = min(spd, self.maxRPM)
			if (rev == 1):
				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(self.ID_Driver, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 178)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()				
			else:
				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(self.ID_Driver, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 170)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()
		elif (spd < 0):
			vel = max(spd, self.minSpeed)
			if (rev == 1):
				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(self.ID_Driver, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 170)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()				
			else:
				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(self.ID_Driver, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 178)
					# print "dir rawData: ", rawData
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()			
		else:
			if (stopMode == 1): # Instantaneous stop.
				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(self.ID_Driver, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 32)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()
			else: # Deceleration stop.
				try:
					time.sleep(self.timeWait)
					rawData = self.MODBUS.execute(self.ID_Driver, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 0)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()

		# -- SPEED
		try:
			if (abs(vel) < 100):
				vel = 0
			time.sleep(self.timeWait) #  + 0.002
			rawData = self.MODBUS.execute(self.ID_Driver, cst.WRITE_SINGLE_REGISTER, self.reg_rotationSpeed_No2, output_value = abs(vel) ) #
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_rotationSpeed_No2 error:" + str(vel) )
			self.countErr_reqSpeed += 1
			print ("countErr_reqSpeed: " + str(self.countErr_reqSpeed) )
			# sys.exit()

	def publish_status(self):
		t_pub = time.time() - self.preTime_pubStatus
		if (t_pub >= self.cycle_pubStatus):
			self.preTime_pubStatus = time.time()
			self.pub_driverRespond.publish(self.driver_respond)

	def config(self, id):
		# -- timeout
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_communicationTimeout, output_value = 500 ) # ms
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_communicationTimeout error")
			sys.exit()	
		# -- count timeOut to warning
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_communicationErrorAlarm, output_value = 3 ) # times
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_communicationErrorAlarm error")	
			sys.exit()	
		# -- 
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_acceleration_No2, output_value = self.accelerationTime ) # 
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_acceleration_No2 error")
			sys.exit()	
		# -- 
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_deceleration_No2, output_value = self.decelerationTime ) # 
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_deceleration_No2 error")
			sys.exit()	
		# -- 
		try:
			time.sleep(self.timeWait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_torqueLimiting_No2, output_value = self.torqueLimiting ) # 
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_torqueLimiting_No2 error")
			sys.exit()	

	def resetAll(self):
		print ("resetAll")
		if (self.statusDriver.WNG == 1 or self.statusDriver.ALARM_OUT1 == 1 or self.statusDriver.ALARM_OUT2 == 1):
			self.resetAlarm(self.ID_Driver)
			print ("resetAll ok")

	def check_query(self):
		if (self.is_readQuery):
			t = time.time() - self.lastTime_checkQuery
			if (t >= self.cycle_checkQuery):
				return 0
			else:	
				return 1
		else:
			return 0

	def check_safety(self):
		if (self.is_readSafety):
			t = time.time() - self.lastTime_checkSafety
			if (t >= self.cycle_checkSafety):
				return 0
			else:	
				return 1
		else:
			return 0


	def run(self):
		time.sleep(0.1)
		self.config(self.ID_Driver)
		time.sleep(0.1)
		self.config(self.ID_Driver)
		time.sleep(0.1)

		self.getStatus_now()
		time.sleep(0.1)
		self.getStatus_now()
		time.sleep(0.1)

		self.resetAll()
		time.sleep(0.1)

		self.getStatus_now()
		time.sleep(0.1)

		while not rospy.is_shutdown():
			# -- check communicate
			if (self.check_query() == 1): #  and  self.check_safety() == 1
				enable_run = 1
				self.driver_respond.status = self.status_run
			else:
				enable_run = 0
				self.driver_respond.status = self.status_stop

			# -- control speed
			if (enable_run):
				self.spin( self.driver_query.modeStop, self.driver_query.rotationSpeed)
			else:
				self.spin(1, 0)

			# -- Task reset
			if (self.driver_query.task == 1):
				self.resetAll()
				self.getStatus()
				self.driver_respond.REV = self.statusDriver.REV
				self.driver_respond.FWD = self.statusDriver.FWD
				self.driver_respond.speed = self.statusDriver.SPEED
				self.driver_respond.alarm_all = self.statusDriver.PRESENT_ALARM
				self.driver_respond.alarm_overload = self.statusDriver.ALARM_OUT2
				self.driver_respond.warning = self.statusDriver.PRESENT_WARNING
				self.driver_respond.message_error = self.errorType_All(self.statusDriver.PRESENT_ALARM)
				self.driver_respond.message_warning = self.errorType_Alarm(self.statusDriver.PRESENT_WARNING)

				# self.publish_status()
				self.pub_driverRespond.publish(self.driver_respond)
				# print ("-----1")
			# -- Task read status
			elif (self.driver_query.task == 2):
				self.getStatus()
				self.driver_respond.REV = self.statusDriver.REV
				self.driver_respond.FWD = self.statusDriver.FWD
				self.driver_respond.speed = self.statusDriver.SPEED
				self.driver_respond.alarm_all = self.statusDriver.PRESENT_ALARM
				self.driver_respond.alarm_overload = self.statusDriver.ALARM_OUT2
				self.driver_respond.warning = self.statusDriver.PRESENT_WARNING
				self.driver_respond.message_error = self.errorType_All(self.statusDriver.PRESENT_ALARM)
				self.driver_respond.message_warning = self.errorType_Alarm(self.statusDriver.PRESENT_WARNING)

				# self.publish_status()
				self.pub_driverRespond.publish(self.driver_respond)
				# print ("-----2")

			else:
				self.getSpeed()
				self.driver_respond.speed = self.statusDriver.SPEED
				# self.publish_status()
				self.pub_driverRespond.publish(self.driver_respond)
				# print ("-----3")

			# print ("runing")
			self.rate.sleep()

		print('Programer stopped')


def main():
	print('Starting main program')

	program = driver()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()
