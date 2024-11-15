#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# blv 200W.
# DATE: 15/11/2021
# AUTHOR: HOANG VAN QUANG - BEE

import rospy
import sys
import time
import threading
import signal

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import Bool

from message_pkg.msg import Driver_query
from message_pkg.msg import Driver_respond
from message_pkg.msg import Loadcell_respond, Loadcell_query

from math import sin , cos , pi , atan2

import struct
import serial
import modbus_tk
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
		+ "       |   "  + str(self.SPEED) )

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
		# ------------------- Load cell
		self.ID_loadcell = rospy.get_param("id_loadcell", 3)
		self.topicSubLoadcell = rospy.get_param("topicSub_loadcell", "/loadcell_query")
		self.topicPubLoadcell = rospy.get_param("topicPub_loadcell", "/loadcell_respond")
		self.origin_weight = rospy.get_param("origin_weight", 100)
		# -------------------
		self.lastTime_readStatus = time.time()
		self.time_readStatus = 2 # s
		self.time_wait = 0.002
		# -- 
		self.accelerationTime = 2 # n*0.1s
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

		self.statusDriver_1 = statusDriver(self.ID_Driver_1)
		self.statusDriver_2 = statusDriver(self.ID_Driver_2)
		self.controlDriver_1 = controlDriver(self.ID_Driver_1, self.revert_1)
		self.controlDriver_2 = controlDriver(self.ID_Driver_2, self.revert_2)
		self.enb_readVel = 0

		# ---- ADD LOADCELL -----------------------
		self.reg_lc1 = 0
		self.reg_lc2 = 1
		self.reg_lc3 = 2

		# ---------------------------------------- Modbus
		print("Modbus run!")
		for i in range(3):
			try:
				self.MODBUS = modbus_rtu.RtuMaster(
					serial.Serial(port= self.PORT, baudrate= self.BAUDRATE, bytesize=8, parity='N', stopbits=1, xonxoff=0)
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
		rospy.init_node('driver_control_all', anonymous=False)
		self.rate = rospy.Rate(1000)

		rospy.Subscriber(self.topicSafety, Bool, self.safety_infoCallback)
		self.safety = Bool()
		self.enable_run = 0
		self.is_readSafety = 0
		self.lastTime_checkSafety = time.time()
		self.cycle_checkSafety = 1 # s

		# -- 
		self.cycle_checkQuery = 0.5 # s
		rospy.Subscriber(self.topicSub_1, Driver_query, self.driverQuery1_infoCallback)
		self.driverQuery_1 = Driver_query()
		self.is_readQuery_1 = 0
		self.lastTime_checkQuery_1 = time.time()

		rospy.Subscriber(self.topicSub_2, Driver_query, self.driverQuery2_infoCallback)
		self.driverQuery_2 = Driver_query()
		self.is_readQuery_2 = 0
		self.lastTime_checkQuery_2 = time.time()
		# --
		self.pub_driverRespond_1 = rospy.Publisher(self.topicPub_1, Driver_respond, queue_size= 50)
		self.driverRespond_1 = Driver_respond()

		self.pub_driverRespond_2 = rospy.Publisher(self.topicPub_2, Driver_respond, queue_size= 50)
		self.driverRespond_2 = Driver_respond()

		# -- Load cell
		self.pub_dataLoadCell = rospy.Publisher(self.topicPubLoadcell, Loadcell_respond, queue_size= 50)
		self.loadcellRespond = Loadcell_respond()

		rospy.Subscriber(self.topicSubLoadcell, Loadcell_query, self.Loadcell_infoCallback)
		self.loadcellQuery = Loadcell_query()
		self.is_loadcellQuery = 0
		self.lastTime_checkloadcellQuery = time.time()

		# -- kiem tra node con chay hay ko.
		self.pub_runing = rospy.Publisher('driver_runing', Int16, queue_size= 50)
		self.driver_runing = Int16()
		self.cycle_pubRuning = 0.1 # s
		self.value_runing = 0
		self.pre_time_runing = time.time()

		self.status_error = 2
		self.status_run = 1
		self.status_stop = 0

	def driverQuery1_infoCallback(self, data):
		self.driverQuery_1 = data
		self.is_readQuery_1 = 1
		self.lastTime_checkQuery_1 = time.time()

	def driverQuery2_infoCallback(self, data):
		self.driverQuery_2 = data
		self.is_readQuery_2 = 1
		self.lastTime_checkQuery_2 = time.time()

	def Loadcell_infoCallback(self, data):
		self.loadcellQuery = data
		self.is_loadcellQuery = 1
		self.lastTime_checkloadcellQuery = time.time()

	def safety_infoCallback(self, data):
		self.safety = data
		self.is_readSafety = 1
		self.lastTime_checkSafety = time.time()

	def pub_topic_runing(self):
		t = (time.time() - self.pre_time_runing)%60 
		if (t > self.cycle_pubRuning):
			self.pre_time_runing = time.time()
			self.driver_runing.data = self.value_runing
			self.pub_driverRespond_2.publish(self.driver_runing)

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

	def int_to_byte(self, val): # int to a bytes
		if val > 255:
			rospy.logerr("int_to_byte: Val error: %s", val)
			val = 255
		elif val < 0:
			rospy.logerr("int_to_byte: Val error: %s", val)
			val = 0
		return struct.pack("B", int(val)) # bytes(chr(int(val)), 'ascii')

	def int_to_bytes(self, val, n):
		ss = b''
		x = 0
		t = 0
		for i in range(0, n):
			t += pow(256, n - i)*x
			x = int((val - t)/pow(256, n - i - 1) )
			ss += self.int_to_byte(x)
		return ss

	def getStatus_all(self, type_):
		# -- get vel, alarm, warning, error, ...
		rawData = 0
		allData = ''
		# ---------------------------------
		t = (time.time() - self.lastTime_readStatus)%60 
		if (t > self.time_readStatus or type_ == 1):
			self.lastTime_readStatus = time.time()
			# -- driver 1
			try:
				time.sleep(self.time_wait)
				rawData = self.MODBUS.execute(self.ID_Driver_1, cst.READ_HOLDING_REGISTERS, self.reg_driverOutputCommand, 1)
				arrData = bin(rawData[0])
				l = len(arrData) - 2
				for i in range(16 - l):
					allData += '0'
				for i in range(l):
					allData += arrData[i + 2]
				# print "len: ", len(allData)
				# print "allData: ", allData
			
				self.statusDriver_1.FWD = int(allData[12])
				self.statusDriver_1.REV = int(allData[11])
				self.statusDriver_1.STOP_MODE = int(allData[10])
				self.statusDriver_1.WNG = int(allData[9])
				self.statusDriver_1.ALARM_OUT1 = int(allData[8])
				self.statusDriver_1.S_BSY = int(allData[7])
				self.statusDriver_1.ALARM_OUT2 = int(allData[3])
				self.statusDriver_1.MOVE = int(allData[2])
				self.statusDriver_1.VA = int(allData[1])
				self.statusDriver_1.TLC = int(allData[0])

			except modbus_tk.modbus.ModbusError as exc:
				print ("Read reg_driverOutputCommand error")
				sys.exit()

			if (self.statusDriver_1.ALARM_OUT1 != 0):
				time.sleep(self.time_wait)
				try:
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.READ_HOLDING_REGISTERS, self.reg_presentAlarm, 1)
					self.statusDriver_1.PRESENT_ALARM = rawData[0]
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_presentAlarm error")
					sys.exit()
											
			else:
				self.statusDriver_1.PRESENT_ALARM = 0

			if (self.statusDriver_1.WNG != 0):
				time.sleep(self.time_wait)
				try:
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.READ_HOLDING_REGISTERS, self.reg_presentWarning, 1)
					self.statusDriver_1.PRESENT_WARNING = rawData[0]
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_presentWarning error")
					sys.exit()
			else:
				self.statusDriver_1.PRESENT_WARNING = 0
				
			# -- driver 2
			try:
				time.sleep(self.time_wait)
				rawData = self.MODBUS.execute(self.ID_Driver_2, cst.READ_HOLDING_REGISTERS, self.reg_driverOutputCommand, 1)
				arrData = bin(rawData[0])
				l = len(arrData) - 2
				for i in range(16 - l):
					allData += '0'
				for i in range(l):
					allData += arrData[i + 2]
				# print "len: ", len(allData)
				# print "allData: ", allData
			
				self.statusDriver_2.FWD = int(allData[12])
				self.statusDriver_2.REV = int(allData[11])
				self.statusDriver_2.STOP_MODE = int(allData[10])
				self.statusDriver_2.WNG = int(allData[9])
				self.statusDriver_2.ALARM_OUT1 = int(allData[8])
				self.statusDriver_2.S_BSY = int(allData[7])
				self.statusDriver_2.ALARM_OUT2 = int(allData[3])
				self.statusDriver_2.MOVE = int(allData[2])
				self.statusDriver_2.VA = int(allData[1])
				self.statusDriver_2.TLC = int(allData[0])

			except modbus_tk.modbus.ModbusError as exc:
				print ("Read reg_driverOutputCommand error")	
				sys.exit()

			if (self.statusDriver_2.ALARM_OUT1 != 0):
				time.sleep(self.time_wait)
				try:
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.READ_HOLDING_REGISTERS, self.reg_presentAlarm, 1)
					self.statusDriver_2.PRESENT_ALARM = rawData[0]
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_presentAlarm error")
					sys.exit()
											
			else:
				self.statusDriver_2.PRESENT_ALARM = 0

			if (self.statusDriver_2.WNG != 0):
				time.sleep(self.time_wait)
				try:
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.READ_HOLDING_REGISTERS, self.reg_presentWarning, 1)
					self.statusDriver_2.PRESENT_WARNING = rawData[0]
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_presentWarning error")
					sys.exit()
			else:
				self.statusDriver_2.PRESENT_WARNING = 0

	def getSpeed_all(self):
		# -- driver 1
		try:
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(self.ID_Driver_1, cst.READ_HOLDING_REGISTERS, self.reg_feedbackSpeed, 1)
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
			print ("Read reg_feedbackSpeed error")
			sys.exit()

		# -- driver 2
		try:
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(self.ID_Driver_2, cst.READ_HOLDING_REGISTERS, self.reg_feedbackSpeed, 1)
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
			print ("Read reg_feedbackSpeed error")
			sys.exit()

		# self.statusDriver_1.showAll()
		# self.statusDriver_2.showAll()

	def stopInstantaneous(self, id):
		rawData = 0
		# ---------------------------------
		try:
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 0)
			# print ("RawData: ", rawData
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_driverInputCommand error")

	def stopDeceleration(self, id):
		rawData = 0
		# ---------------------------------
		try:
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 32)
			# print ("RawData: ", rawData
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_driverInputCommand error")

	def resetAlarm(self, id):
		rawData = 0
		# ---------------------------------
		try:
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_resetAlarm, output_value = 1)
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read resetAlarm error")
			sys.exit()
		# ---------------------------------
		try:
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_resetAlarm, output_value = 0)
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read resetAlarm error")
			sys.exit()

	def spin_1(self, stopMode, spd):
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
		rev = self.controlDriver_1.REVERT

		if (spd > 0):
			vel = min(spd, self.maxRPM)
			if (rev == 1):
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 178)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")	
					sys.exit()
			else:
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 170)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()
		elif (spd < 0):
			vel = max(spd, self.minRPM)
			if (rev == 1):
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 170)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()				
			else:
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 178)
					# print "dir rawData: ", rawData
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()
		else:
			if (stopMode == 1): # Instantaneous stop.
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 32)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()
			else: # Deceleration stop.
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 0)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()

		# -- SPEED
		try:
			if (abs(vel) < 100):
				vel = 0
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_rotationSpeed_No2, output_value = abs(vel) ) #
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_rotationSpeed_No2 error")
			sys.exit()

	def spin_2(self, stopMode, spd):
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
		rev = self.controlDriver_2.REVERT

		if (spd > 0):
			vel = min(spd, self.maxRPM)
			if (rev == 1):
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 178)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()				
			else:
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 170)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()
		elif (spd < 0):
			vel = max(spd, self.minRPM)
			if (rev == 1):
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 170)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()				
			else:
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 178)
					# print "dir rawData: ", rawData
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()			
		else:
			if (stopMode == 1): # Instantaneous stop.
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 32)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()
			else: # Deceleration stop.
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 0)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()

		# -- SPEED
		try:
			if (abs(vel) < 100):
				vel = 0
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_rotationSpeed_No2, output_value = abs(vel) ) #
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_rotationSpeed_No2 error")
			sys.exit()

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
		# -- driver 1
		if (spd_1 > 0):
			vel_1 = min(spd_1, self.maxRPM)
			if (rev_1 == 1):
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 178)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")	
					sys.exit()				
			else:
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 170)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()
		elif (spd_1 < 0):
			vel_1 = max(spd_1, self.minRPM)
			if (rev_1 == 1):
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 170)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()				
			else:
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 178)
					# print "dir rawData: ", rawData
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()			
		else:
			if (stopMode_1 == 1): # Instantaneous stop.
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 32)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()
			else: # Deceleration stop.
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 0)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()

		# -- driver 2
		if (spd_2 > 0):
			vel_2 = min(spd_2, self.maxRPM)
			if (rev_2 == 1):
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 178)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()				
			else:
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 170)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()
		elif (spd_2 < 0):
			vel_2 = max(spd_2, self.minRPM)
			if (rev_2 == 1):
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 170)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()				
			else:
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 178)
					# print "dir rawData: ", rawData
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()			
		else:
			if (stopMode_2 == 1): # Instantaneous stop.
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 32)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()
			else: # Deceleration stop.
				try:
					time.sleep(self.time_wait)
					rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_driverInputCommand, output_value = 0)
				except modbus_tk.modbus.ModbusError as exc:
					print ("Read reg_driverInputCommand error")
					sys.exit()

		# -- -- SPEED
		# -- driver 1
		try:
			if (abs(vel_1) < 100):
				vel_1 = 0
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(self.ID_Driver_1, cst.WRITE_SINGLE_REGISTER, self.reg_rotationSpeed_No2, output_value = abs(vel_1) ) #
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_rotationSpeed_No2 error")
			sys.exit()
			
		# -- driver 2
		try:
			if (abs(vel_2) < 100):
				vel_2 = 0
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(self.ID_Driver_2, cst.WRITE_SINGLE_REGISTER, self.reg_rotationSpeed_No2, output_value = abs(vel_2) ) #
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_rotationSpeed_No2 error")
			sys.exit()

	def config(self, id):
		print ("Config ID ", id)
		# -- timeOut
		try:
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_communicationTimeout, output_value = 200 ) # ms
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_communicationTimeout error")
			sys.exit()	
		# -- count timeOut to warning
		try:
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_communicationErrorAlarm, output_value = 3 ) # times
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_communicationErrorAlarm error")	
			sys.exit()	
		# -- 
		try:
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_acceleration_No2, output_value = self.accelerationTime ) # 
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_communicationErrorAlarm error")	
			sys.exit()	
		# -- 
		try:
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_deceleration_No2, output_value = self.decelerationTime ) # 
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_communicationErrorAlarm error")	
			sys.exit()	
		# -- 
		try:
			time.sleep(self.time_wait)
			rawData = self.MODBUS.execute(id, cst.WRITE_SINGLE_REGISTER, self.reg_torqueLimiting_No2, output_value = self.torqueLimiting ) # 
		except modbus_tk.modbus.ModbusError as exc:
			print ("Read reg_communicationErrorAlarm error")
			sys.exit()	

	def resetAll(self):
		print ("Run reset")
		if (self.statusDriver_1.WNG == 1 or self.statusDriver_1.ALARM_OUT1 == 1 or self.statusDriver_1.ALARM_OUT2 == 1):
			self.resetAlarm(self.ID_Driver_1)
			print ("reset ID_Driver_1")

		if (self.statusDriver_2.WNG == 1 or self.statusDriver_2.ALARM_OUT1 == 1 or self.statusDriver_2.ALARM_OUT2 == 1):
			self.resetAlarm(self.ID_Driver_2)
			print ("reset ID_Driver_2")

	def check_query(self):
		if (self.is_readQuery_1 == 1 or self.is_readQuery_2 == 1):
			t = time.time() - self.lastTime_checkQuery_1
			if (t >= self.cycle_checkQuery):
				return 0
			else:
				return 1
		else:
			return 0 	

	def pubRespondLoadcell(self, status, weight, arrBit):
		loadcellRespond = Loadcell_respond()
		loadcellRespond.header.stamp = rospy.Time.now()
		loadcellRespond.status = status
		if status != -1:
			mess = '|'
			loadcellRespond.weight = weight
			if arrBit[0] == '1':
				mess = mess + ' LOST LOADCELL 4 |'
			if arrBit[1] == '1':
				mess = mess + ' LOST LOADCELL 3 |'
			if arrBit[2] == '1':
				mess = mess + ' LOST LOADCELL 2 |'
			if arrBit[3] == '1':
				mess = mess + ' LOST LOADCELL 1 |'
			
			if mess == '|':
				loadcellRespond.message_info = 'ALL LOADCELL STILL ALIVE'
			else:
				loadcellRespond.message_info = mess

		else:
			loadcellRespond.weight = 0.0
			loadcellRespond.message_info = 'ERROR'
		
		self.pub_dataLoadCell.publish(loadcellRespond)
		
	def read_weight(self):
		try:
			time.sleep(0.4)
			rawData1 = self.MODBUS.execute(self.ID_loadcell, cst.READ_HOLDING_REGISTERS, self.reg_lc1, 1)
			time.sleep(0.4)
			rawData2 = self.MODBUS.execute(self.ID_loadcell, cst.READ_HOLDING_REGISTERS, self.reg_lc2, 1)
			time.sleep(0.4)
			rawData3 = self.MODBUS.execute(self.ID_loadcell, cst.READ_HOLDING_REGISTERS, self.reg_lc3, 1)
			print ("rawData3: ", rawData3)
			print("data1: " + str(rawData1[0]) + " | data2: " + str(rawData2[0]) )
			print("data1: " + str(self.int_to_bytes(rawData1[0], 2) ) + " | data2: " + str(self.int_to_bytes(rawData2[0], 2) ) + " | data3: " + str(rawData3[0]))
			st = b''
			a1 = b''
			a2 = b''
			a1 = self.int_to_bytes(rawData1[0], 2)
			# print(a1)

			a10 = self.int_to_byte(a1[0])
			# print(a10)
			a11 = self.int_to_byte(a1[1])
			# print(a11)

			a2 = self.int_to_bytes(rawData2[0], 2)
			a20 = self.int_to_byte(a2[0])
			a21 = self.int_to_byte(a2[1])

			st += a11
			st += a10
			# print(st)
			st += a21
			st += a20
			value = struct.unpack('f', st)[0]
			temp = '000' + format(rawData3[0], "b")
			temp = temp[len(temp)-4:]
			# print(temp)

			print ("weight: ", value)
			self.pubRespondLoadcell(1, value, temp)

		except:
			print ("Read LOADCELL reg_feedback error")
			self.pubRespondLoadcell(-1, 0.0, '')

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
		self.config(self.ID_Driver_1)
		time.sleep(0.1)
		self.config(self.ID_Driver_2)
		time.sleep(0.1)

		self.getStatus_all(1)
		time.sleep(0.1)

		self.resetAll()
		time.sleep(0.1)

		self.getStatus_all(1)
		time.sleep(0.1)

		while not rospy.is_shutdown():
			# -- check communicate
			if (self.check_query() == 1): #  and  self.check_safety() == 1
				self.enable_run = 1
				self.driverRespond_1.status = self.status_run
				self.driverRespond_2.status = self.status_run
				self.value_runing = 1
			else:
				self.enable_run = 0
				self.driverRespond_1.status = self.status_stop
				self.driverRespond_2.status = self.status_stop
				self.value_runing = 0

			if self.loadcellQuery.task == 1:
				self.read_weight()

			else:
				# -- control speed
				t = time.time() - self.lastTime_checkloadcellQuery
				if t > 2.0:
					if (self.enable_run):
						self.spin_all(self.driverQuery_1.rotationSpeed, self.driverQuery_2.rotationSpeed, self.driverQuery_1.modeStop, self.driverQuery_2.modeStop)
					else:
						# print ("here!")
						self.spin_all(0, 0, 1, 1)

					# -- Task reset
					if (self.driverQuery_1.task == 1):
						self.resetAll()
						self.getStatus_all(0)
						self.getSpeed_all()
						# -- driver 1
						self.driverRespond_1.REV = self.statusDriver_1.REV
						self.driverRespond_1.FWD = self.statusDriver_1.FWD
						self.driverRespond_1.speed = self.statusDriver_1.SPEED
						self.driverRespond_1.alarm_all = self.statusDriver_1.PRESENT_ALARM
						self.driverRespond_1.alarm_overload = self.statusDriver_1.ALARM_OUT2
						self.driverRespond_1.warning = self.statusDriver_1.PRESENT_WARNING
						self.driverRespond_1.message_error = self.errorType_All(self.statusDriver_1.PRESENT_ALARM)
						self.driverRespond_1.message_warning = self.errorType_Alarm(self.statusDriver_1.PRESENT_ALARM)
						# -- driver 2
						self.driverRespond_2.REV = self.statusDriver_2.REV
						self.driverRespond_2.FWD = self.statusDriver_2.FWD
						self.driverRespond_2.speed = self.statusDriver_2.SPEED
						self.driverRespond_2.alarm_all = self.statusDriver_2.PRESENT_ALARM
						self.driverRespond_2.alarm_overload = self.statusDriver_2.ALARM_OUT2
						self.driverRespond_2.warning = self.statusDriver_2.PRESENT_WARNING
						self.driverRespond_2.message_error = self.errorType_All(self.statusDriver_2.PRESENT_ALARM)
						self.driverRespond_2.message_warning = self.errorType_Alarm(self.statusDriver_2.PRESENT_ALARM)

						self.pub_driverRespond_1.publish(self.driverRespond_1)
						self.pub_driverRespond_2.publish(self.driverRespond_2)

					# -- Task read status
					elif (self.driverQuery_1.task == 2):
						self.getStatus_all(0)
						self.getSpeed_all()
						# -- driver 1
						self.driverRespond_1.REV = self.statusDriver_1.REV
						self.driverRespond_1.FWD = self.statusDriver_1.FWD
						self.driverRespond_1.speed = self.statusDriver_1.SPEED
						self.driverRespond_1.alarm_all = self.statusDriver_1.PRESENT_ALARM
						self.driverRespond_1.alarm_overload = self.statusDriver_1.ALARM_OUT2
						self.driverRespond_1.warning = self.statusDriver_1.PRESENT_WARNING
						self.driverRespond_1.message_error = self.errorType_All(self.statusDriver_1.PRESENT_ALARM)
						self.driverRespond_1.message_warning = self.errorType_Alarm(self.statusDriver_1.PRESENT_ALARM)
						# -- driver 2
						self.driverRespond_2.REV = self.statusDriver_2.REV
						self.driverRespond_2.FWD = self.statusDriver_2.FWD
						self.driverRespond_2.speed = self.statusDriver_2.SPEED
						self.driverRespond_2.alarm_all = self.statusDriver_2.PRESENT_ALARM
						self.driverRespond_2.alarm_overload = self.statusDriver_2.ALARM_OUT2
						self.driverRespond_2.warning = self.statusDriver_2.PRESENT_WARNING
						self.driverRespond_2.message_error = self.errorType_All(self.statusDriver_2.PRESENT_ALARM)
						self.driverRespond_2.message_warning = self.errorType_Alarm(self.statusDriver_2.PRESENT_ALARM)

						self.pub_driverRespond_1.publish(self.driverRespond_1)
						self.pub_driverRespond_2.publish(self.driverRespond_2)

					else:
						self.getSpeed_all()
						self.driverRespond_1.speed = self.statusDriver_1.SPEED
						self.driverRespond_2.speed = self.statusDriver_2.SPEED

						self.pub_driverRespond_1.publish(self.driverRespond_1)
						self.pub_driverRespond_2.publish(self.driverRespond_2)

			# self.pub_topic_runing() # kiem tra node con chay hay ko.
			self.rate.sleep()

		print('Programer stopped')


def main():
	print('Starting main program')

	program = driver()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()
