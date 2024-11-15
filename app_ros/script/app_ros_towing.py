#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Developer: Phùng Quý Dương(Archie Phùng)
Company: STI Viet Nam
Date: 21/05/2024

 >> Hiển thị vùng cảnh báo nhỏ và to trên màn hình: Đê define sau
 
"""

import sys
from math import sin , cos , pi , atan2
import time
import threading
import signal
import json

import os
import re  
import subprocess
import argparse
from datetime import datetime

import sqlite3

# -- 
from app_interface_towing import *
# from fakePlanSti_v2 import fakePlan
from makePlanAuto import fakePlan

class Program(threading.Thread):
	def __init__(self, threadID):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.shutdown_flag = threading.Event()
		# --
		self.name_card = rospy.get_param("name_card", "wlo2")
		self.name_card = "wlo2"

		self.address_traffic = rospy.get_param("address_traffic", "172.21.15.224")
		#self.address_traffic = "192.168.1.92"
		self.pre_timePing = time.time()

		self.pathDataPose = '/home/stivietnam/catkin_ws/src/app_ros/config/dataSetPose.json'
		# --
		rospy.init_node('app_ros', anonymous=False)
		self.rate = rospy.Rate(20)

		self.app = QApplication(sys.argv)
		self.welcomeScreen = WelcomeScreen()
		screen = self.app.primaryScreen()

		size = screen.size()
		print('Size: %d x %d' % (size.width(), size.height()))

		self.widget = QtWidgets.QStackedWidget()
		self.widget.addWidget(self.welcomeScreen)
		self.widget.setFixedHeight(540)
		self.widget.setFixedWidth(1024)
		# --
		self.valueLable = valueLable()
		self.statusColor = statusColor()
		# -- 
		self.is_exist = 1
		# -----------------------------------------------------------
		# -- Break
		rospy.Subscriber("/enable_brake", Bool, self.callback_brakeControl) 
		self.status_brake = Bool()

		# -- Driver1
		rospy.Subscriber("/driver1_respond", Driver_respond, self.callback_driver1) 
		self.driver1_respond = Driver_respond()

		# -- Driver2
		rospy.Subscriber("/driver2_respond", Driver_respond, self.callback_driver2) 
		self.driver2_respond = Driver_respond()

		# -- HC
		rospy.Subscriber("/HC_info", HC_info, self.callback_HC) 
		self.HC_info = HC_info()

		# -- Main
		rospy.Subscriber("/POWER_info", POWER_info, self.callback_Main) 
		self.main_info = POWER_info()

		# -- CPD Boad
		# rospy.Subscriber("/lift_status", Lift_status, self.callback_OC_board) # lay thong tin trang thai mach dieu khien ban nang.
		# self.OC_status = Lift_status()

		# -- Status Port
		rospy.Subscriber("/status_port", Status_port, self.callback_statusPort) 
		self.status_port = Status_port()

		# ------------------------------
		# -- data nav
		# rospy.Subscriber("/nav350_data", Nav350_data, self.callback_nav350) 
		# self.nav350_data = Nav350_data()

		# -- data safety Nano Scan
		rospy.Subscriber("/safety_zone", Zone_lidar_2head, self.callback_safetyNS3) 
		self.safety_ns3 = Zone_lidar_2head()

		# -- Pose robot
		rospy.Subscriber("/robotPose_lidarLOC", PoseStamped, self.callback_robotPose) 
		self.robotPose_loc = PoseStamped()

		# -- Traffic cmd
		rospy.Subscriber("/server_cmdRequest", Server_cmdRequest, self.callback_server_cmdRequest)
		self.server_cmdRequest = Server_cmdRequest()

		# -- Traffic cmd
		rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.NN_cmdRequest_callback) 
		self.NN_cmdRequest = NN_cmdRequest()

		# -- 
		rospy.Subscriber('/request_move', LineRequestMove, self.callback_moveRequest)
		self.dataRequestMove = LineRequestMove() 
		self.is_moveRequest = False

		# -- Pose robot
		rospy.Subscriber("/NN_infoRequest", NN_infoRequest, self.callback_NN_infoRequest) 
		self.NN_infoRequest = NN_infoRequest()

		# -- info AGV
		rospy.Subscriber("/NN_infoRespond", NN_infoRespond, self.infoAGV_callback) 
		self.NN_infoRespond = NN_infoRespond()

		# -- info move
		rospy.Subscriber("/move_respond", Status_goal_control, self.goalControl_callback)
		self.status_goalControl = Status_goal_control() # sub from move_base

		# -- Launch
		rospy.Subscriber("/status_launch", Status_launch, self.callback_statusLaunch)
		self.status_launch = Status_launch()

		# -- status Lidarloc
		rospy.Subscriber("/localizationcontroller/out/localizationcontroller_result_message_0502", LocalizationControllerResultMessage0502, self.callback_statusLOC)
		self.status_loc = LocalizationControllerResultMessage0502()

		# -- Info Move
		# rospy.Subscriber("/navigation_respond", Navigation_respond, self.callback_navigationRespond)
		# self.navigation_respond = Navigation_respond()

		# -- CPU info
		rospy.Subscriber("/nuc_info", Nuc_info, self.callback_nuc)
		self.Nuc_info = Nuc_info()

		# -----------------------------------------------------------
		rospy.Subscriber("/cancelMission_status", Int16, self.callBack_cancelMission)
		self.cancelMission_status = Int16()
		# --
		self.pub_cancelMission = rospy.Publisher("/cancelMission_control", Int16, queue_size = 4)
		self.cancelMission_control = Int16()
		# --
		self.pub_button = rospy.Publisher("/app_button", App_button, queue_size = 4)
		self.app_button = App_button()
		self.pre_app_setColor = App_color()

		# --
		self.name_agv = ""
		self.ip_agv = ""
		# --
		self.modeRuning = 0
		self.modeRun_launch = 0
		self.modeRun_byhand = 1
		self.modeRun_auto = 2
		self.modeRun_byhand_tryTarget = 3
		self.modeRun_cancelMission = 5

		# self.app_button.bt_speaker = 1
		self.listIdSetpose = []
		self.dataDicSetPose = {}
		self.isSettingPose = 1
		self.savetimeSetpose = rospy.Time.now()
		self.savetimeDeletepose = rospy.Time.now()
		self.savetimeAddpose = rospy.Time.now()

		self.listpath = []
		self.saveTime_readIP = rospy.get_time()

		self.loadDataSetPose()

		self.fakePl = fakePlan()
		# self.pr0 = self.fakePl.QT[0]    # 
		# self.pr1 = self.fakePl.QT[1]    # 
		# self.pr2 = self.fakePl.QT[2]    # 

		for plan in self.fakePl.QT:
			self.getListFullPath(plan)
		
		for plan in self.fakePl.QT:
			self.valueLable.list_indexTarget.append(self.getIndexofPointTarget(plan, self.valueLable.listFullPath))

		print(self.valueLable.list_indexTarget)

		# -- 
		self.step = 0

		self.search_radius = 300
		self.isSetPose_success = 0

	def getIndexofPointTarget(self, pr, list_path):
		index = 0
		for i in range(0, len(list_path)):
			if pr.target_x == list_path[i].endPoint.x and pr.target_y == list_path[i].endPoint.y:
				index = i
				break
		return index
	
	def getListFullPath(self, pr):
		for path in pr.pathInfo:
			ipath = infoPath()
			ipath.typeLine = path.typePath
			ipath.startPoint = Point(path.pointOne.position.x, path.pointOne.position.y, 0.0)
			ipath.endPoint = Point(path.pointSecond.position.x, path.pointSecond.position.y, 0.0)
			if path.typePath == TypeLine.BEZIERLINE.value:
				ipath.midPoint = Point(path.pointMid.position.x, path.pointMid.position.y, 0.0)
			self.valueLable.listFullPath.append(ipath)

	
	def callback_brakeControl(self, data):
		self.status_brake = data
		
	def callback_driver1(self, data):
		self.driver1_respond = data

	def callback_driver2(self, data):
		self.driver2_respond = data

	def callback_HC(self, data):
		self.HC_info = data

	def callback_Main(self, data):
		self.main_info = data
		
	def callback_OC_board(self, data):
		self.OC_status = data

	def callback_statusPort(self, data):
		self.status_port = data

	def goalControl_callback(self, data):
		self.status_goalControl = data
		
	def callBack_cancelMission(self, data):
		self.cancelMission_status = data

	def callback_nav350(self, data):
		self.nav350_data = data

	def callback_statusLOC(self, data):
		self.status_loc = data

	def callback_safetyNAV(self, data):
		self.safety_NAV = data

	def callback_safetyNS3(self, data):
		self.safety_ns3 = data

	def callback_robotPose(self, data):
		self.robotPose_loc = data

	def callback_server_cmdRequest(self, data):
		self.server_cmdRequest = data

	def NN_cmdRequest_callback(self, data):
		self.NN_cmdRequest = data	

	def callback_NN_infoRequest(self, data):
		self.NN_infoRequest = data

	def infoAGV_callback(self, data):
		self.NN_infoRespond = data	

	def callback_statusLaunch(self, data):
		self.status_launch = data

	def callback_navigationRespond(self, data):
		self.navigation_respond = data

	def callBack_cancelMission(self, data):
		self.cancelMission_status = data

	def callback_moveRequest(self, data):
		self.dataRequestMove = data

	def callback_nuc(self, data):
		self.Nuc_info = data

	def convert_position(self, distance, angle):
		x = 0
		y = 0
		x = distance*cos(angle)
		y = distance*sin(angle)
		# y = distance*cos(angle)
		# x = distance*sin(angle)
		return x, y

	def ping_traffic(self, address):
		try:
			ping = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=True)
			# print(ping)
			vitri = str(ping).find("time")
			time_ping = str(ping)[(vitri+5):(vitri+9)]
			# print (time_ping)
			return str(float(time_ping))
		except Exception:
			return '-1'

	def get_ipAuto(self, name_card): # name_card : str()
		try:
			address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
			print ("address: ", address)
			return address
		except Exception:
			return "-1"

	def get_ipAndMac(self):
		if (rospy.get_time() - self.saveTime_readIP > 60):
			self.saveTime_readIP = rospy.get_time()
			self.valueLable.lbv_ip = self.get_ipAuto(self.name_card)
			self.valueLable.lbv_mac = self.get_MAC(self.name_card)
			self.valueLable.lbv_namePc = self.get_hostname()

	def run_screen(self):
		self.widget.show()
		try:
			# print ("run 1")
			sys.exit(self.app.exec_())
			# print ("run 2")
		except:
			pass
			# print("Exiting 1")
		self.is_exist = 0

	def kill_app(self):
		self.welcomeScreen.out()
		self.is_exist = 0

	# --
	def get_MAC(self, name_card): # name_card : str()
		try:
			MAC = ''
			output = os.popen("ip addr show {}".format(name_card) ).read()
			pos1 = str(output).find('link/ether ') # tuyet doi ko sua linh tinh.
			pos2 = str(output).find(' brd')   # tuyet doi ko sua linh tinh.

			if (pos1 >= 0 and pos2 > 0):
				MAC = str(output)[pos1+11:pos2]

			print ("MAC: ", MAC)
			return MAC
		except Exception:
			return "-1"
	# --
	def get_qualityWifi(self, name_card): # int
		try:
			quality_data = '0'
			output = os.popen("iwconfig {}".format(name_card)).read()
			pos_quality = str(output).find('Link Quality=')
			# -
			if pos_quality >= 0:
				quality_data = str(output)[pos_quality+13:pos_quality+15]
			# print ("quality_data: ", int(quality_data) )
			# -
			return int(quality_data)
		except Exception:
			return 0
	# ---
	def get_hostname(self):
		try:
			output = os.popen("hostname").read()
			# print ("output: ", output)
			leng = len(output)
			hostname = str(output)[0:leng-1]
			print ("hostname: ", hostname)
			return hostname
		except Exception:
			return "-1"

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def limitAngle(self, angle_in): # - rad
		qua_in = self.euler_to_quaternion(angle_in)
		angle_out = self.quaternion_to_euler(qua_in)
		return angle_out

	def convert_errorAll(self, val):
		switcher={
			0:'AGV Hoạt Động Bình Thường',
			1:'AGV Đang Di Chuyển Theo Odom',
			311:'Mất kết nối với Mạch STI-RTC',			
			361:'Mất Kết Nối Với Mạch STI-CPD', # 
			351:'Mất Kết Nối Với Mạch STI-HC', # 
			352:'Không Giao Tiếp CAN Với Mạch STI-HC', # 
			341:'Mất Kết Nối Với Mạch STI-OC', #
			342:'Mất Cổng USB của USB của Mạch STI-OC', # 
			343:'Không Giao Tiếp CAN Với Mạch STI-OC', # 
			344:'Không Giao Tiếp Với Mạch STI-OC1', # 
			345:'Không Giao Tiếp Với Mạch STI-OC2', # 
			346:'Không Giao Tiếp Với Mạch STI-OC3', # 
			323:'Mạng CAN Không Gửi Được', # 
			321:'Mất Kết Nối Với Mạch STI-Main', # 
			322:'Mất Cổng USB của USB của Mạch STI-Main', # 
			251:'Mất Kết Nối Với Driver1', # 
			252:'Lỗi Động Cơ Số 1', # 
			261:'Mất Kết Nối Với Driver2', # 
			262:'Lỗi Động Cơ Số 2', # 
			231:'Mất Kết Nối Với Cảm Biến Góc', # 
			232:'Mất Cổng USB của Cảm Biến IMU', # 
			240:'Mất dữ liệu Safety',
			241:'Lỗi Định Vị: Matching', # 
			242:'Lỗi Định Vị: Localization', # 
			221:'Mất Kết Nối Lidar Localization', # 
			181:'LoadCell-Ket Noi', # 
			182:'LoadCell-Dau noi', # 
			183:'LoadCell-USB', # 
			184:'Quá Tải 700kg', # 
			222:'Mất Tọa Độ Robot', # 
			141:'Lỗi Không Chạm Được Cảm Biến Bàn Nâng', # 
			121:'Trạng Thái Dừng Khẩn - EMG', # 
			122:'AGV Bị Chạm Blsock', #
			272:'Không Phát Hiện Được Đủ Gương', #
			281:'Mất TF Parking', #
			282:'Mất Gói GoalControl', #
			291:'AGV nằm ngoài đường dẫn',
			292:'Lỗi Gói GoalControl',
			293:'1.Hãy Điều Khiển AGV Vào Đường Dẫn, 2.Nhấn Bắt Đầu',#
			441:'AGV Đã Di Chuyển Hết Điểm', #
			442:'AGV Đang Dừng Để Nhường Đường Cho AGV Khác', # 
			477:'Không Có Kệ Tại Vị Trí', # 
			411:'Vướng Vật Cản - Di Chuyển Giữa Các Điểm', #
			412:'Vướng Vật Cản - Di Chuyển Vào Vị Trí Kệ', # 
			431:'AGV Không Giao Tiếp Với Phần Mềm Traffic', #
			451:'Điện Áp Của AGV Đang Rất Thấp', # 
			452:'AGV Không Sạc Được Pin', #
			453:'Không Phát Hiện Được Đủ Gương', #
			454:'Đang Khởi Tạo Lại Vị Trí của AGV', #
			443:'CPU > 99%'

		}
		return switcher.get(val, 'UNK')

	def show_job(self, val):
		job_now = 'Không\nXác Định'
		switcher={
			0:'...', #
			1:'Kiểm Tra Lại Nhiệm Vụ', # 
			2:'Thực Hiện Nhiệm Vụ Trước', # 
			3:'Kiểm Tra Trạng Thái Kệ',
			4:'Di Chuyển Ra Khởi Vị Trí', # 
			5:'Di Chuyển Giữa Các Điểm', #
			6:'Tạm Dừng Di Chuyển', # 
			7:'Thực Hiện Nhiệm Vụ Sau', # 
			8:'Đợi Lệnh Mới', # 
			9:'Đợi Hoàn Thành Lệnh Cũ', #
			10:'Đợi Tín Hiệu Bắt Đầu Từ Người Dùng',
			20:'Chế Độ Bằng Tay', # 
			30:'AGV đang gặp lỗi ', # 
			50:'Kiểm Tra Vị Trí Trả Hàng', # 
			
            #-- Towing 
			-1:'Di chuyển giữa các điểm',
			-2:'Di chuyển đến vị trí',
			-3:'Chờ xác nhận tiếp tục di chuyển',
			-4:'Đợi lệnh mới',
			-5:'Chế Độ Bằng Tay', # 
			-6:'Chế Độ Tự Động', # 
		}
		return switcher.get(val, job_now)

	def show_misson(self, val):
		job_now = 'Không\nXác Định'
		switcher={
			0:'...', #
			65:'Nâng Kệ', # 
			1:'Nâng Kệ', # 
			66:'Hạ Kệ', #
			2:'Hạ Kệ', #
			6:'Sạc Pin', # 
			10:'Hạ Kệ\nSạc Pin' # 
		}
		return switcher.get(val, job_now)

	def callServiceInitializeAtPose(self, x, y, theta, sRadius):
		try:
			serClient = rospy.ServiceProxy('/LocInitializeAtPose', LocInitializeAtPoseSrv)
			rospy.loginfo("Generated [], sending LocInitializeAtPose request...")
			resp = serClient(x, y, theta, sRadius)
			return resp.success

		except rospy.ServiceException as e:
			rospy.logwarn(e)
		
		return False

	def callServiceGetLocalizationStatus(self):
		try:
			serClient = rospy.ServiceProxy('/LocGetLocalizationStatus', LocGetLocalizationStatusSrv)
			rospy.loginfo("Generated [], sending LocGetLocalizationStatus request...")
			resp = serClient()
			if resp.success:
				return resp.locstatus
			
		except rospy.ServiceException as e:
			rospy.logwarn(e)

		return -1

	def callServiceAutoStartSavePose(self):
		try:
			serClient = rospy.ServiceProxy('/LocAutoStartSavePose', LocAutoStartSavePoseSrv)
			rospy.loginfo("Generated [], sending LocAutoStartSavePose request...")
			resp = serClient()
			print(resp.success)

		except rospy.ServiceException as e:
			rospy.logwarn(e)

	def controlColor(self):
		# -- HC_info
		# self.statusColor.lbc_safety_ahead = self.HC_info.zone_sick_ahead
		# self.statusColor.lbc_safety_behind = self.HC_info.zone_sick_behind
		# -- safetyZone
		self.statusColor.lbc_safety_ahead = self.safety_ns3.zone_ahead
		self.statusColor.lbc_safety_behind = self.safety_ns3.zone_behind
		# --
		self.statusColor.lbc_button_clearError = self.main_info.stsButton_reset
		self.statusColor.lbc_button_power = self.main_info.stsButton_power
		self.statusColor.lbc_emg = self.main_info.EMC_status
		self.statusColor.lbc_blsock = self.HC_info.vacham

		# -- Port
		self.statusColor.lbc_port_rtc    = self.status_port.board
		self.statusColor.lbc_port_driverLeft  = self.status_port.motorLeft
		self.statusColor.lbc_port_driverRight  = self.status_port.motorRight

		if self.status_port.motorLeft == 1 and self.status_port.motorRight == 1:
			self.statusColor.lbc_port_rs485 = 1
		else:
			self.statusColor.lbc_port_rs485 = 0

		self.statusColor.lbc_port_lidar = self.status_port.lidar
		self.statusColor.lbc_port_magLine = self.status_port.magLine

		# -- Toyo
		self.statusColor.lbc_toyoReadBit0 = self.HC_info.vacham               # Archie 
		# self.statusColor.lbc_limit_up = self.OC_status.sensorUp.data
		# self.statusColor.lbc_limit_down = self.OC_status.sensorDown.data
		# self.statusColor.lbc_detect_lifter = self.OC_status.sensorLift.data

	def loadDataSetPose(self):
		listID = []
		# Opening JSON file
		try:
			with open(self.pathDataPose, 'r') as openfile:
				# Reading from json file
				json_object = json.load(openfile)

			for p in json_object:
				id = p[5:]
				listID.append(id)

			if self.valueLable.listIDSetpose != listID:
				self.dataDicSetPose = json_object
				# print(self.dataDicSetPose)
				self.valueLable.isUpdateListIdSetpose = True
				self.valueLable.listIDSetpose = listID

			# print("List ban dau", self.valueLable.listIDSetpose)
		except:
			pass
	
	def updateDataSetPose(self):
		if self.welcomeScreen.flag_deletePose == 1:
			self.welcomeScreen.flag_deletePose = 0
			selecId = self.welcomeScreen.cb_listAddressSetPose2.currentText()
			if selecId == '':
				self.statusColor.lbc_deletePose = -1
				self.savetimeDeletepose = rospy.Time.now()
				return
			
			else:
				del self.dataDicSetPose['pose_' + selecId]
				self.valueLable.listIDSetpose.remove(selecId)

			self.valueLable.isUpdateListIdSetpose = True
			# print(self.dataDicSetPose)

			try:
				with open(self.pathDataPose, "w") as outfile: 
					json.dump(self.dataDicSetPose, outfile)
			except:
				print("Convert dict to json Error")

		if self.welcomeScreen.flag_addPose == 1:
			self.welcomeScreen.flag_addPose = 0

			addID = self.welcomeScreen.valueLable.lbv_tryTarget_d
			if addID == 0:
				print("ID bang 0")
				return
			
			else:
				# check xem id được thêm có trùng với các id khác ko ?
				for id in self.valueLable.listIDSetpose:
					if addID == int(id):
						print("ID bij trungf list")
						return
				
				self.valueLable.listIDSetpose.append(str(addID))
				dx = self.welcomeScreen.valueLable.lbv_tryTarget_x
				dy = self.welcomeScreen.valueLable.lbv_tryTarget_y
				dz = 0.0
				quat = self.euler_to_quaternion(radians(self.welcomeScreen.valueLable.lbv_tryTarget_r))

				drx = round(quat.x, 3)
				dry = round(quat.y, 3)
				drz = round(quat.z, 3)
				drw = round(quat.w, 3)

				self.dataDicSetPose['pose_'+ str(addID)] = {'rw': drw, 'rx': drx, 'ry': dry, 'rz': drz, 'x': dx, 'y': dy, 'z': dz}
				# print(self.dataDicSetPose)

				try:
					with open(self.pathDataPose, "w") as outfile: 
						json.dump(self.dataDicSetPose, outfile)
				except:
					print("Convert dict to json Error")

				self.valueLable.isUpdateListIdSetpose = True

	def controlSetPose(self):
		if self.welcomeScreen.statusButton.bt_setpose == 0 or self.isSettingPose == 1:
			return

		self.isSettingPose = 1
		selecId = self.welcomeScreen.cb_listAddressSetPose.currentText()
		if selecId == '':
			self.statusColor.lbc_setpose = -1
			self.savetimeSetpose = rospy.Time.now()
			return 
		
		dataPose = self.dataDicSetPose.get('pose_' + selecId)
		#print(dataPose)
		if dataPose == None:
			self.statusColor.lbc_setpose = -1
			self.savetimeSetpose = rospy.Time.now()
			return

		dX = int(dataPose['x']*1000)
		dY = int(dataPose['y']*1000)
		euler = euler_from_quaternion((0, 0, dataPose['rz'], dataPose['rw']))
		deg = degrees(euler[2])
		if deg > 180.:
			deg = 360. - deg
			
		dG = int(deg*1000)
		fradius = 800
		self.search_radius = 300
		self.isSetPose_success = 0

		while self.search_radius <= 2000:
			call = self.callServiceInitializeAtPose(dX, dY, dG, self.search_radius)
			self.search_radius += 100

			if call:
				self.isSetPose_success = 1
				break
		
		if self.isSetPose_success == 1:
			self.statusColor.lbc_setpose = 1
		else:
			self.statusColor.lbc_setpose = -1

		self.savetimeSetpose = rospy.Time.now()

	def getIndexofPathFollow(self, id_follow, plan):
		index = 0
		for i, path in enumerate(plan.pathInfo):
			if id_follow == path.pathID:
				index = i
				break
		
		return index
		
	def getListPath(self):
		if self.dataRequestMove.enable:
			# --

			for index, target in enumerate(self.valueLable.list_indexTarget):
				x = self.valueLable.listFullPath[target].endPoint.x
				y = self.valueLable.listFullPath[target].endPoint.y

				if self.dataRequestMove.target_x == x and self.dataRequestMove.target_y == y:
					self.valueLable.typeTarget = index

					# --
			
			pr = self.fakePl.QT[self.valueLable.typeTarget]
			k = self.getIndexofPathFollow(self.status_goalControl.ID_follow, pr)
			self.valueLable.lbv_idPathFollow = self.status_goalControl.ID_follow
			self.valueLable.lbv_pathVelocity = round(pr.pathInfo[k].velocity,1)
			self.valueLable.lbv_pathSafety = round(pr.pathInfo[k].fieldSafety,1)
			
			if pr.pathInfo[k].typePath == 1:
				self.valueLable.lbv_typeLineFollow = 'Đ.Thẳng'
			elif pr.pathInfo[k].typePath == 3:
				self.valueLable.lbv_typeLineFollow = 'Đ.Cong'

			self.valueLable.lbv_xTarget = self.dataRequestMove.target_x
			self.valueLable.lbv_ytarget = self.dataRequestMove.target_y
			self.valueLable.lbv_rTarget = self.dataRequestMove.target_z

			self.valueLable.lbv_idtarget = self.fakePl.QT[self.valueLable.typeTarget].pathInfo[-1].pathID
			
			if self.listpath != self.dataRequestMove.pathInfo:
				self.listpath = self.dataRequestMove.pathInfo

				self.valueLable.listPath = []

				for p in self.listpath:
					ipath = infoPath()

					if p.pointSecond.position.x == self.dataRequestMove.target_x and p.pointSecond.position.y == self.dataRequestMove.target_y:
						ipath.isTarget = True
						
					if p.typePath == 1:
						ipath.typeLine = TypeLine.STRAIGHTLINE
						ipath.startPoint = Point(p.pointOne.position.x, p.pointOne.position.y, 0.)
						ipath.endPoint = Point(p.pointSecond.position.x, p.pointSecond.position.y, 0.)
						self.valueLable.listPath.append(ipath)

					elif p.typePath == 3:
						ipath.typeLine = TypeLine.BEZIERLINE
						ipath.startPoint = Point(p.pointOne.position.x, p.pointOne.position.y, 0.)
						ipath.midPoint = Point(p.pointMid.position.x, p.pointMid.position.y, 0.)
						ipath.endPoint = Point(p.pointSecond.position.x, p.pointSecond.position.y, 0.)
						self.valueLable.listPath.append(ipath)

		# else:
		# 	self.valueLable.listPath = []

	def find_element(self, value_find, list_in):
		lenght = len(list_in)
		for i in range(lenght):
			if (value_find == list_in[i]):
				return 1
		return 0
	
	def controlAll(self):
		# -- Mode show
		if (self.NN_infoRespond.mode == 0):   # - launch
			self.valueLable.modeRuning = self.modeRun_launch

		elif (self.NN_infoRespond.mode == 1): # -- md_by_hand
			self.valueLable.modeRuning = self.modeRun_byhand

		elif (self.NN_infoRespond.mode == 2): # -- md_auto
			self.valueLable.modeRuning = self.modeRun_auto

		# -- Battery
		if (self.main_info.charge_current > 0.1):
			self.statusColor.lbc_battery = 4
		else:
			if (self.main_info.voltages < 23.5):
				self.statusColor.lbc_battery = 3
			elif (self.main_info.voltages >= 23.5 and self.main_info.voltages < 24.5):
				self.statusColor.lbc_battery = 2
			else:
				self.statusColor.lbc_battery = 1

		bat = round(self.main_info.voltages, 1)
		if bat > 25.5:
			bat = 25.5
		self.valueLable.lbv_battery = "  " + str(bat)

		# -- status AGV
		self.statusColor.cb_status = self.NN_infoRespond.status

		lg_err = len(self.NN_infoRespond.listError)
		self.valueLable.listError = []
		if (lg_err == 0):
			self.valueLable.listError.append( self.convert_errorAll(0))       # agv hoat dong binh thuong
		else:
			length = len(self.valueLable.list_logError)
			if length > 15:
				self.valueLable.list_logError = []
			# -
			for i in range(lg_err):
				self.valueLable.listError.append( self.convert_errorAll(self.NN_infoRespond.listError[i]) )
				# -
				if self.NN_infoRespond.listError[i] < 100:
					self.valueLable.listError.append( self.convert_errorAll(self.NN_infoRespond.listError[i]) )
		
		if self.welcomeScreen.statusButton.bt_clearError == 1:
			if self.find_element(291, self.NN_infoRespond.listError) == 1:
				self.valueLable.status_show_nav = 1
		
		if self.welcomeScreen.statusButton.bt_doNextPlan2 == 1:
			self.valueLable.status_show_nav = 0

		if self.find_element(291, self.NN_infoRespond.listError) == 1:
			self.valueLable.lbv_statusPath = 1
		else:
			self.valueLable.lbv_statusPath = 0

		# -
		self.valueLable.lbv_name_agv = self.NN_infoRequest.name_agv
		# -- Ping
		deltaTime_ping = (time.time() - self.pre_timePing)%60
		if (deltaTime_ping > 2.0):
			self.pre_timePing = time.time()
			self.valueLable.lbv_pingServer = self.ping_traffic(self.address_traffic)
			# -
			self.valueLable.lbv_qualityWifi = self.get_qualityWifi(self.name_card)

		# -- 
		self.valueLable.lbv_coordinates_x = round(self.robotPose_loc.pose.position.x, 3)
		self.valueLable.lbv_coordinates_y = round(self.robotPose_loc.pose.position.y, 3)
		angle = self.quaternion_to_euler(self.robotPose_loc.pose.orientation)
		
		if angle < 0:
			angle_robot = 2*pi + angle
		else:
			angle_robot = angle

		self.valueLable.lbv_coordinates_r = round(degrees(angle_robot), 3)
		# --
		# self.valueLable.lbv_route_target = str(self.NN_cmdRequest.target_id) + "\n" + str(round(self.NN_cmdRequest.target_x, 3)) + "\n" + str(round(self.NN_cmdRequest.target_y, 3)) + "\n" + str(round(degrees(self.NN_cmdRequest.target_z), 2)) + "\n" + str(round(self.NN_cmdRequest.offset, 3))
		# # # -
		# if len(self.NN_cmdRequest.list_id) >= 5:
		# 	self.valueLable.lbv_route_point0 = str(self.NN_cmdRequest.list_id[0]) + "\n" + str(round(self.NN_cmdRequest.list_x[0], 3)) + "\n" + str(round(self.NN_cmdRequest.list_y[0], 3)) + "\n" + str(self.NN_cmdRequest.list_speed[0])
		# 	self.valueLable.lbv_route_point1 = str(self.NN_cmdRequest.list_id[1]) + "\n" + str(round(self.NN_cmdRequest.list_x[1], 3)) + "\n" + str(round(self.NN_cmdRequest.list_y[1], 3)) + "\n" + str(self.NN_cmdRequest.list_speed[1])
		# 	self.valueLable.lbv_route_point2 = str(self.NN_cmdRequest.list_id[2]) + "\n" + str(round(self.NN_cmdRequest.list_x[2], 3)) + "\n" + str(round(self.NN_cmdRequest.list_y[2], 3)) + "\n" + str(self.NN_cmdRequest.list_speed[2])
		# 	self.valueLable.lbv_route_point3 = str(self.NN_cmdRequest.list_id[3]) + "\n" + str(round(self.NN_cmdRequest.list_x[3], 3)) + "\n" + str(round(self.NN_cmdRequest.list_y[3], 3)) + "\n" + str(self.NN_cmdRequest.list_speed[3])
		# 	self.valueLable.lbv_route_point4 = str(self.NN_cmdRequest.list_id[4]) + "\n" + str(round(self.NN_cmdRequest.list_x[4], 3)) + "\n" + str(round(self.NN_cmdRequest.list_y[4], 3)) + "\n" + str(self.NN_cmdRequest.list_speed[4])
		
		# self.valueLable.lbv_route_job1 = str(self.NN_cmdRequest.before_mission)
		# self.valueLable.lbv_route_job2 = str(self.NN_cmdRequest.after_mission)

		# self.valueLable.lbv_route_job1_mean = self.show_misson(self.NN_cmdRequest.before_mission)
		# self.valueLable.lbv_route_job2_mean = self.show_misson(self.NN_cmdRequest.after_mission)
		if self.status_loc.map_match_status == 90:
			self.valueLable.lbv_map_match_status = 'Cao'
		elif self.status_loc.map_match_status == 60:
			self.valueLable.lbv_map_match_status = 'Vừa'
		elif self.status_loc.map_match_status == 30:
			self.valueLable.lbv_map_match_status = 'Thấp'
		else:
			self.valueLable.lbv_map_match_status = 'Không'

		if self.status_loc.loc_status == 10:
			self.valueLable.lbv_loc_status = 'OK'
		elif self.status_loc.loc_status == 20:
			self.valueLable.lbv_loc_status = 'Warn'
		elif self.status_loc.loc_status == 30:
			self.valueLable.lbv_loc_status = 'NG'
		else:
			self.valueLable.lbv_loc_status = 'Lỗi'

		self.valueLable.lbv_route_message = self.dataRequestMove.mess
		self.valueLable.lbv_jobRuning = self.show_job(self.NN_infoRespond.process)
		self.valueLable.lbv_jobDoing = self.NN_infoRespond.process
		self.valueLable.is_recvNNinfoRespond = 1

		# -- 
		self.valueLable.lbv_goalFollow_id = str(self.status_goalControl.ID_follow)
		self.valueLable.lbv_errorPath = str(round(self.status_goalControl.path_error, 3))

		# -- Launch
		self.valueLable.percentLaunch = self.status_launch.persent
		self.valueLable.lbv_launhing = self.status_launch.notification
		self.valueLable.lbv_numberLaunch = self.status_launch.position

		# lidar loc 
		self.updateDataSetPose()
		self.controlSetPose()
		if self.isSettingPose:
			dentalTime = rospy.Time.now() - self.savetimeSetpose
			if dentalTime.to_sec() > 2.:
				self.isSettingPose = 0
				self.statusColor.lbc_setpose = 0

		self.getListPath()

		# -- Driver 
		self.valueLable.lbv_notification_driver1 = self.driver1_respond.message_error
		self.valueLable.lbv_notification_driver2 = self.driver2_respond.message_error
		self.valueLable.lbv_velLeft = str(self.driver1_respond.speed)
		self.valueLable.lbv_velRight = str(self.driver2_respond.speed)

		# -- Nuc info
		self.valueLable.lbv_cpu = str(self.Nuc_info.cpu_usage) + " %"
		self.valueLable.lbv_cputemp = str(self.Nuc_info.cpu_temp) +" "+ str(chr(176))+"C"
		self.valueLable.lbv_ram = str(self.Nuc_info.ram_usage)+"/"+ str(self.Nuc_info.ram_total)

		if self.Nuc_info.wifi_signal < 0:
			if self.Nuc_info.wifi_signal >= -60:
				self.valueLable.lbv_wifi = "Tốt"
			elif self.Nuc_info.wifi_signal >= -70 and self.Nuc_info.wifi_signal < -60:
				self.valueLable.lbv_wifi = "Trung bình"
			elif self.Nuc_info.wifi_signal >= -80 and self.Nuc_info.wifi_signal < -70:
				self.valueLable.lbv_wifi = "Kém"
			elif self.Nuc_info.wifi_signal < -80:
				self.valueLable.lbv_wifi = "Rất Kém"

		if self.Nuc_info.wifi_signal == 0 or self.Nuc_info.wifi_quality == 0:
			self.valueLable.lbv_wifi = "Mất kết nối"

		self.valueLable.lbv_uptime = self.Nuc_info.uptime

	def readButton(self):
		# -- 
		self.app_button.bt_cancelMission = self.welcomeScreen.statusButton.bt_cancelMission
		self.app_button.bt_passAuto 	 = self.welcomeScreen.statusButton.bt_passAuto
		self.app_button.bt_passHand 	 = self.welcomeScreen.statusButton.bt_passHand
		self.app_button.bt_setting 		 = self.welcomeScreen.statusButton.bt_setting
		self.app_button.bt_clearError 	 = self.welcomeScreen.statusButton.bt_clearError
		# --
		self.app_button.bt_moveHand 	  = self.welcomeScreen.statusButton.bt_moveHand
		# --
		self.app_button.bt_charger	= self.welcomeScreen.statusButton.bt_charger
		self.app_button.bt_speaker  = self.welcomeScreen.statusButton.bt_speaker
		self.app_button.bt_brake = self.welcomeScreen.statusButton.bt_disableBrake
		self.app_button.bt_setpose	= self.welcomeScreen.statusButton.bt_setpose
		# self.app_button.bt_lifter	= self.welcomeScreen.statusButton.bt_lifter
		# self.app_button.bt_auxiliaryLights	= self.welcomeScreen.statusButton.bt_auxiliaryLights
		# -- 
		self.app_button.ck_remote = self.welcomeScreen.statusButton.ck_remote
		# self.app_button.bt_resetFrameWork = self.welcomeScreen.statusButton.bt_resetFrameWork
		# -
		self.app_button.vs_speed = self.welcomeScreen.statusButton.vs_speed
		# -
		self.app_button.bt_doNextPlan = self.welcomeScreen.statusButton.bt_doNextPlan
		self.app_button.bt_doNextPlan2 = self.welcomeScreen.statusButton.bt_doNextPlan2
		self.app_button.bt_doNextPlanClicked = self.welcomeScreen.statusButton.bt_doNextPlanClicked

		# - 
		self.app_button.soundtype = self.welcomeScreen.statusButton.soundtype
		self.app_button.ledtype = self.welcomeScreen.statusButton.ledtype
		self.app_button.toyobit = self.welcomeScreen.statusButton.toyobit

	def run(self):
		# -- 
		self.valueLable.lbv_ip = self.get_ipAuto(self.name_card)
		self.valueLable.lbv_mac = self.get_MAC(self.name_card)
		self.valueLable.lbv_namePc = self.get_hostname()
		while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()) and (self.is_exist == 1):
			self.get_ipAndMac()
			self.controlAll()
			self.controlColor()
			# --
			self.readButton()
			self.pub_button.publish(self.app_button)

			if self.cancelMission_status.data == 1:
				self.welcomeScreen.statusButton.bt_cancelMission = 0
				self.cancelMission_control.data = 0

			if self.welcomeScreen.statusButton.bt_cancelMission == 1:
				self.cancelMission_control.data = 1

			self.pub_cancelMission.publish(self.cancelMission_control)

			# ----------------------
			self.welcomeScreen.valueLable = self.valueLable
			self.welcomeScreen.statusColor = self.statusColor
			# -- 
			self.welcomeScreen.robotPoseNow = self.robotPose_loc.pose
			# -
			# print("Giá trị của biến update cb", self.valueLable.isUpdateListIdSetpose)
			# print("List hiện tại là:", self.valueLable.listIDSetpose)
			self.rate.sleep()

		self.is_exist = 0
		self.kill_app()

		print('Thread #%s stopped' % self.threadID)

	# def check_wifi(self):
	# 	while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()) and (self.is_exist == 1):
	# 		# -- Ping
	# 		self.valueLable.lbv_pingServer = self.ping_traffic(self.address_traffic)
	# 		# -
	# 		self.valueLable.lbv_qualityWifi = self.get_qualityWifi(self.name_card)
	# 		self.time.sleep(1.5)

class ServiceExit(Exception):
	"""
	Custom exception which is used to trigger the clean exit
	of all running threads and the main program.
	"""
	pass
 
def service_shutdown(signum, frame):
	print('Caught signal %d' % signum)
	raise ServiceExit

def main():
	# Register the signal handlers
	signal.signal(signal.SIGTERM, service_shutdown)
	signal.signal(signal.SIGINT, service_shutdown)

	print('Starting main program')

	# Start the job threads
	try:
		thread1 = Program(1)
		thread1.start()

		# Keep the main thread running, otherwise signals are ignored.
		thread1.run_screen()
		thread1.is_exist = 0

	except ServiceExit:
		thread1.shutdown_flag.set()
		thread1.join()
		print('Exiting main program')
 
if __name__ == '__main__':
	main()
