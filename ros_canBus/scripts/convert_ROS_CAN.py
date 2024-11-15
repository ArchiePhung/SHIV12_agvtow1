#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: HOANG VAN QUANG - BEE
# DATE: 03/08/2022

# from message_pkg.msg import *
from message_pkg.msg import CAN_send, CAN_status, CAN_received, CPD_read, CPD_write
from sti_msgs.msg import *
from geometry_msgs.msg import Twist
import time
import rospy
from std_msgs.msg import Int8
from ros_canBus.msg import *

class CAN_ROS():
	def __init__(self):
		print("ROS Initial: CONVERT CAN BOARD!")
		rospy.init_node('CAN_ROS', anonymous=False)
		self.rate = rospy.Rate(50)

		# ------------- PARAMETER ------------- #
		self.ID_RTC  = rospy.get_param('ID_RTC', 1)
		self.ID_RTC  = 1

		self.ID_HC   = rospy.get_param('ID_HC', 2)
		self.ID_HC   = 2

		self.ID_MAIN = rospy.get_param('ID_MAIN', 3)
		self.ID_MAIN = 3

		self.ID_MC = rospy.get_param('ID_MC', 4)
		self.ID_MC = 4
		# ------------- ROS ------------- #
		# --
		self.pub_statusMain = rospy.Publisher("/POWER_info", POWER_info, queue_size = 10)
		self.status_Main = POWER_info()
		# -- 
		self.pub_statusHC = rospy.Publisher("/HC_info", HC_info, queue_size = 20)
		self.status_HC = HC_info()
		# -- 
		self.pub_statusOC = rospy.Publisher("/lift_status", Lift_status, queue_size = 10)
		self.status_OC = Lift_status()

		self.pub_statusToyoLeft = rospy.Publisher("/toyoLeft_info", toyo_info, queue_size = 10)
		self.status_ToyoLeft = toyo_info()

		self.pub_statusToyoRight = rospy.Publisher("/toyoRight_info", toyo_info, queue_size = 10)
		self.status_ToyoRight = toyo_info()

		self.pub_statusEncoder = rospy.Publisher("/encoder_respond", EncoderData, queue_size = 10)
		self.status_encoder = EncoderData()

		# - SUBCRIBER
		# ------------- CAN ------------- #
		# rospy.Subscriber("/CAN_status", CAN_status, self.statusCAN_callback)
		# self.CAN_status = CAN_status()

		rospy.Subscriber("/CAN_received", CAN_received, self.CAN_callback)
		self.data_receivedCAN = CAN_received()

		self.pub_sendCAN = rospy.Publisher("/CAN_send", CAN_send, queue_size= 40)
		self.data_sendCan = CAN_send()
		self.frequence_sendCAN = 14. # - Hz
		self.saveTime_sendCAN = time.time()
		self.sort_send = 0

		# ------------- Control ------------- #
		rospy.Subscriber("/POWER_request", POWER_request, self.controlMain_callback)
		self.is_powerRequest = 0
		self.data_controlMain = POWER_request()

		rospy.Subscriber("/HC_request", HC_request, self.controlHC_callback)
		self.data_controlHC = HC_request()

		rospy.Subscriber("/HC_fieldRequest1", Int8, self.controlFieldSick1_callback)
		self.data_controlFieldSick1 = Int8()

		rospy.Subscriber("/HC_fieldRequest2", Int8, self.controlFieldSick2_callback)
		self.data_controlFieldSick2 = Int8()

		rospy.Subscriber("/encoder_request", Int8, self.controlEncoder_callback)
		self.data_controlMC = Int8()

		# ------------- VAR ------------- #

	# -------------------
	def controlFieldSick1_callback(self, data):
		self.data_controlFieldSick1 = data

	def controlFieldSick2_callback(self, data):
		self.data_controlFieldSick2 = data

	def controlHC_callback(self, data):
		self.data_controlHC = data

	def controlMain_callback(self, data):
		self.data_controlMain = data
		self.is_powerRequest = 1

	def controlEncoder_callback(self, data):
		self.data_controlMC = data

	# def controlOC_callback(self, data):
	# 	self.data_controlOC = data

	# ------------------- 
	def CAN_callback(self, data):
		self.data_receivedCAN = data
		# -- RECEIVED CAN
		self.analysisFrame_receivedCAN()

	def statusCAN_callback(self, data):
		self.CAN_status = data

	def convert_4byte_int(self, byte0, byte1, byte2, byte3):
		int_out = 0
		int_out = byte0 + byte1*256 + byte2*256*256 + byte3*256*256*256
		if int_out > pow(2, 32)/2.:
			int_out = int_out - pow(2, 32)
		return int_out

	def convert_16bit_int(self, bitArr):
		int_out = 0
		for i in range(16):
			int_out += bitArr[i]*pow(2, i)
		return int_out

	def getByte_fromInt16(self, valueIn, pos):
		byte1 = int(valueIn/256)
		byte0 =  valueIn - byte1*256
		if (pos == 0):
			return byte0
		else:
			return byte1

	def getBit_fromInt8(self, value_in, pos):
		bit_out = 0
		value_now = value_in
		for i in range(8):
			bit_out = value_now%2
			value_now = value_now/2
			if (i == pos):
				return bit_out

			if (value_now < 1):
				return 0		
		return 0

	def getBit_fromInt16(self, value_in, pos):
		bit_out = 0
		value_now = value_in
		for i in range(16):
			bit_out = value_now%2
			value_now = value_now/2
			if (i == pos):
				return bit_out

			if (value_now < 1):
				return 0		
		return 0

	def syntheticFrame_sendCAN(self):
		# -- HC
		if (self.sort_send == 0):
			self.data_sendCan.id = self.ID_RTC
			self.data_sendCan.byte0 = self.ID_HC
			self.data_sendCan.byte1 = self.data_controlHC.RBG1
			self.data_sendCan.byte2 = self.data_controlHC.RBG2
			self.data_sendCan.byte3 = self.data_controlFieldSick2.data
			self.data_sendCan.byte4 = 0
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 1

		# -- Main
		elif (self.sort_send == 1):
			if self.is_powerRequest == 1:
				self.data_sendCan.id = self.ID_RTC
				self.data_sendCan.byte0 = self.ID_MAIN
				self.data_sendCan.byte1 = self.data_controlMain.sound_on
				self.data_sendCan.byte2 = self.data_controlMain.sound_type
				self.data_sendCan.byte3 = self.data_controlMain.charge
				self.data_sendCan.byte4 = self.data_controlMain.EMC_reset
				self.data_sendCan.byte5 = self.data_controlMain.EMC_write
				self.data_sendCan.byte6 = self.data_controlMain.led_type
				self.data_sendCan.byte7 = 0

			self.sort_send = 2

		# -- OC
		elif (self.sort_send == 2):
			self.data_sendCan.id = self.ID_RTC
			self.data_sendCan.byte0 = self.ID_MC
			self.data_sendCan.byte1 = self.data_controlMC.data
			self.data_sendCan.byte2 = 0
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 0
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0

			self.sort_send = 0

	def analysisFrame_receivedCAN(self):
		# -- HC
		if self.data_receivedCAN.idSend == self.ID_HC:
			self.status_HC.status 			= 0
			# self.status_HC.zone_sick_ahead  = self.data_receivedCAN.byte4
			# self.status_HC.zone_sick_behind = self.data_receivedCAN.byte5
			self.status_HC.vacham 			= self.data_receivedCAN.byte6
			self.pub_statusHC.publish(self.status_HC)
			# print ("--- HC")
			self.status_ToyoRight.status = 1 if ((self.data_receivedCAN.byte4>>1)&1) == 0 else 0
			self.status_ToyoRight.in1 = 1 if ((self.data_receivedCAN.byte4)&1) == 0 else 0
			self.status_ToyoRight.in2 = 1 if ((self.data_receivedCAN.byte4>>2)&1) == 0 else 0
			self.pub_statusToyoRight.publish(self.status_ToyoRight)

			self.status_ToyoLeft.status = 1 if ((self.data_receivedCAN.byte5>>1)&1) == 0 else 0
			self.status_ToyoLeft.in1 = 1 if ((self.data_receivedCAN.byte5)&1) == 0 else 0
			self.status_ToyoLeft.in2 = 1 if ((self.data_receivedCAN.byte5>>2)&1) == 0 else 0
			self.pub_statusToyoLeft.publish(self.status_ToyoLeft)

		# -- Main
		elif self.data_receivedCAN.idSend == self.ID_MAIN:
			self.status_Main.voltages 	 = self.convert_4byte_int(self.data_receivedCAN.byte0, self.data_receivedCAN.byte1, 0, 0)/10.
			self.status_Main.voltages_analog = 0

			self.status_Main.charge_current  = self.convert_4byte_int(self.data_receivedCAN.byte2, self.data_receivedCAN.byte3, 0, 0)
			self.status_Main.charge_analog   = 0

			# print (self.status_Main.stsButton_reset)
			self.status_Main.stsButton_reset = self.data_receivedCAN.byte4

			self.status_Main.stsButton_power = self.data_receivedCAN.byte5
			self.status_Main.EMC_status 	 = self.data_receivedCAN.byte6
			self.status_Main.CAN_status   	 = self.data_receivedCAN.byte7
			self.pub_statusMain.publish(self.status_Main)
			# print ("--- --- Main")

		elif self.data_receivedCAN.idSend == self.ID_MC:

			right_val = self.data_receivedCAN.byte0|(self.data_receivedCAN.byte1<<8)|(self.data_receivedCAN.byte2<<16)|(self.data_receivedCAN.byte3<<24)
			left_val = self.data_receivedCAN.byte4|(self.data_receivedCAN.byte5<<8)|(self.data_receivedCAN.byte6<<16)|(self.data_receivedCAN.byte7<<24)

			if left_val >= 0x80000000:
				left_val -= 0x100000000
			if right_val >= 0x80000000:
				right_val -= 0x100000000

			self.status_encoder.encoder_left_val = left_val
			self.status_encoder.encoder_right_val = right_val

			# print("left: ", left_val, " | right: ", right_val)
			self.pub_statusEncoder.publish(self.status_encoder)


	def try_run(self):
		val = 100
		print ("Bit0", self.getBit_fromInt8(val, 0))
		print ("Bit1", self.getBit_fromInt8(val, 1))
		print ("Bit2", self.getBit_fromInt8(val, 2))
		print ("Bit3", self.getBit_fromInt8(val, 3))
		print ("Bit4", self.getBit_fromInt8(val, 4))
		print ("Bit5", self.getBit_fromInt8(val, 5))
		print ("Bit6", self.getBit_fromInt8(val, 6))
		print ("Bit7", self.getBit_fromInt8(val, 7))

	def string_arry(self):
		arr_str = "0123456"
		print ("OUT0: ", arr_str[2:4])
		print ("OUT1: ", arr_str[0:2])
		print ("OUT2: ", arr_str[:2])
		print ("OUT3: ", arr_str[:-3])
		print ("OUT4: ", arr_str[-2:0])
		print ("OUT5: ", arr_str[2:0])

	def run(self):
		while not rospy.is_shutdown():
			# -- SEND CAN
			delta_time = (time.time() - self.saveTime_sendCAN)%60 
			if (delta_time > 1/self.frequence_sendCAN):
				self.saveTime_sendCAN = time.time()
				self.syntheticFrame_sendCAN()
				self.pub_sendCAN.publish(self.data_sendCan)

			self.rate.sleep()

def main():
	print('Program starting')
	program = CAN_ROS()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()



