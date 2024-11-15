#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Developer: Phùng Quý Dương
Company: STI Viet Nam
Date  : 21/05/2023
Update:
    + Chương trình get đường dẫn từ web server

	+ Thêm các mã lỗi: 
	  - Lỗi ra khỏi đường dẫn:    lấy code của Hoàng: 
	     + status.error = error  ## 0: bt, 1: (AGV ra khỏi đường dẫn), 2: lỗi hệ thống: OK
		    >> lõi 1; chuyển chế đọ bằng tay và điều khiern AGV vào, sau đó chuyển tự động : Chưa nhìn thấy đường dẫn khi điều khiển
			     + Nếu trước đó, agv tạm dừng, thì bắt đầu lại

			>> Lỗi 2: nhấn xóa lỗi, gửi enb = 0 hoặc chuyển chế độ bằng tay sang tự động : đã sửa, nhưng chưa test
		 + status.safety = safety   # 0: bình thường | 1 (có vật cản); OK

	+ Test bằng tay các âm loa : chưa test

	+ Chế độ tự động: Điều khiển âm loa/ led theo yêu cầu : chưa test 

	+ Thêm thông báo nút xóa lỗi đã được nhấn với lỗi động cơ: 

Update 23/7/2024:
    + Change queue size of topic request_move from 1024 -> 10
	+ xuát led từ topic Power_request

Update 23/8/2024
    + Thêm quy trình Xử lý lỗi khi AGV gặp lỗi Matching hoặc Định vị (241 và 242)

Update 28/8/2024:
    + Kiểm tra lai tọa độ của AGV sau khi tự động lấy lại map

Update 29/10/2024:
    + Thêm chương trình AGV tiếp tục chạy khi lidar loc lỗi
       -> Matching != none: AGV chạy theo trạng thái lls
       -> matching == None: AGV chạy theo odom, khi định vị được lại thì ok
       -> Lỗi localizaion => AGV chạy theo odom, setpose trong khi chạy
       -> Lọc vị trí AGV họat động tốt theo mảng => dùng queue
       -> Khi mới khởi động lên sẽ ko check lõi này

Update 31/10/2024
	+ Setpose again if robot pass 1.5m and not still localization and matching:
	    - If AGV move simple done: OK
	    - IF AGV is transfering: Test

	+ Cảnh báo lỗi nếu CPU máy > 90%: Test
	+ Tắt và bật lại Lidar Loc nếu nó đang dừng chờ quá bao nhiêu phút: Chờ
	+ Nhận tín hiệu Toyo ở mọi bước trong quy trình: Test
	+ Filter Pose final for Setpose => use Queue

"""

import roslib

import sys
import time
from decimal import *
import math
import rospy

# -- add 19/01/2022
import subprocess
import re
import os

from sti_msgs.msg import *

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped, PoseWithCovarianceStamped
from std_msgs.msg import Int16, Int8, Bool
from nav_msgs.msg import Odometry
from message_pkg.msg import *
from sensor_msgs.msg import Imu
from ros_canBus.msg import *
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees
from sick_lidar_localization.msg import LocalizationControllerResultMessage0502
from sick_lidar_localization.srv import LocInitializeAtPoseSrv
from robot_localization.srv import SetPose

#from fakePlanSti_v2 import *
from makePlanAuto import *
# from Queue_Simple import Queue

#--------------------------------------------------------------------------------- ROS
class ros_control():
	def __init__(self):
		rospy.init_node('stiControl', anonymous=False)
		self.rate = rospy.Rate(30)

		# -- fake Qt
		self.fakePl = fakePlan()
		self.numQt = len(self.fakePl.QT)
		print("num QT", self.numQt)
		self.qtNow = 5
		self.completed_moveSimple = 0      # bao da den dich.

		# -- Queue initialize with 10 items
		# self.pose_queue = Queue(10)
		
		# -- cancel mission
		rospy.Subscriber("/cancelMission_control", Int16, self.callback_cancelMission)
		self.cancelMission_control = Int16()		
		self.flag_cancelMission = 0
		self.status_cancel = 0

		self.pub_cancelMission = rospy.Publisher("/cancelMission_status", Int16, queue_size=100)	
		self.cancelMission_status = Int16()		
		# -------------- Cac ket noi ngoai vi:
		# -- Reconnect
		rospy.Subscriber("/status_reconnect", Status_reconnect, self.callback_reconnect)
		self.status_reconnect = Status_reconnect()

		# -- MAIN - POWER
		rospy.Subscriber("/POWER_info", POWER_info, self.callback_main) 
		self.pub_requestMain = rospy.Publisher("/POWER_request", POWER_request, queue_size=100)	
		self.main_info = POWER_info()
		self.power_request = POWER_request()
		self.timeStampe_main = rospy.Time.now()
		self.voltage = 24.5

		self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.velPs2 = Twist()

		# -- HC 82
		rospy.Subscriber("/HC_info", HC_info, self.callback_hc) # lay thong tin trang thai cua node va cam bien sick an toan.
		self.pub_HC = rospy.Publisher("/HC_request", HC_request, queue_size=100)	# dieu khien den bao va den ho tro camera
		self.HC_info = HC_info()
		self.HC_request = HC_request()
		self.timeStampe_HC = rospy.Time.now()

		self.pub_LED = rospy.Publisher("/led_query", Int8, queue_size= 20)
		self.led_query = Int8()

		# -- toyo info
		rospy.Subscriber("/toyoLeft_info", toyo_info, self.callback_toyoLeft) # Lấy tín hiệu toyo
		self.toyoLeft_info = toyo_info()
		self.saveData_toyoLeft = False

		rospy.Subscriber("/toyoRight_info", toyo_info, self.callback_toyoRight) # Lấy tín hiệu toyo
		self.toyoRight_info = toyo_info()
		self.saveData_toyoRight = False

		# -- App
		rospy.Subscriber("/app_button", App_button, self.callback_appButton) # lay thong tin trang thai nut nhan tren man hinh HMI.
		self.app_button = App_button()

		# -- data safety NAV
		rospy.Subscriber("/safety_NAV", Int8, self.safety_NAV_callback) 
		self.safety_NAV = Int8()

		# -- data nav
		# rospy.Subscriber("/nav350_data", Nav350_data, self.nav350_callback) 
		# self.nav350_data = Nav350_data()

		# -- Data LidaLoc
		rospy.Subscriber("/localizationcontroller/out/localizationcontroller_result_message_0502", LocalizationControllerResultMessage0502, self.lidarLoc_callback) 
		self.lidarLoc_status = LocalizationControllerResultMessage0502()

		# -------------- Cac node thuat toan dieu khien.

		self.pub_moveReq = rospy.Publisher("/request_move", LineRequestMove, queue_size = 10)   # 1024
		self.request_move = LineRequestMove()
		self.saveTime_pubRequestMove = rospy.get_time()
		self.rate_pubReqMove = 10

		# -- Communicate with Server
		rospy.Subscriber("/NN_cmdRequest", NN_cmdRequest, self.callback_cmdRequest)
		self.NN_cmdRequest = NN_cmdRequest()

		self.pub_infoRespond = rospy.Publisher("/NN_infoRespond", NN_infoRespond, queue_size=100)
		self.NN_infoRespond = NN_infoRespond()

		# -- Safety Zone
		rospy.Subscriber("/safety_zone", Zone_lidar_2head, self.callback_safetyZone)
		self.zoneRobot = Zone_lidar_2head()
		self.is_readZone = 0
		self.timeStampe_safetyZone = rospy.Time.now()

		# -- Odometry - POSE
		rospy.Subscriber('/robotPose_lidarLOC', PoseStamped, self.callback_getPose, queue_size = 100)
		self.robotPose_lidarLOC = PoseStamped()

		# -- Port physical
		rospy.Subscriber("/status_port", Status_port, self.callback_port)
		self.status_port = Status_port()

		# -- parking
		rospy.Subscriber("/parking_respond", Parking_respond, self.callback_parking)
		self.parking_status = Parking_respond()
		self.parking_poseTarget = Pose()
		self.is_parking_status = 0
		self.flag_requirResetparking = 0 # do truoc do co loi Parking: ko nhin thay Tag

		self.pub_parking = rospy.Publisher("/parking_request", Parking_request, queue_size= 20)
		self.enb_parking = 0
		self.parking_poseBefore = Pose()
		self.parking_poseAfter = Pose()
		self.parking_offset = 0.0
		# -- backward
		self.flag_requirBackward = 0
		self.completed_backward = 0
		self.backward_x = 0.0
		self.backward_y = 0.0
		self.backward_z = 0.0

		rospy.Subscriber("/NN_infoRequest", NN_infoRequest, self.callback_infoRequest)
		self.NN_infoRequest = NN_infoRequest()
		self.timeStampe_server = rospy.Time.now()

		rospy.Subscriber("/move_respond", Status_goal_control, self.callback_goalControl)
		self.status_goalControl = Status_goal_control() # sub from move_base
		self.timeStampe_statusGoalControl = rospy.Time.now()

		rospy.Subscriber("/driver1_respond", Driver_respond, self.callback_driver1)
		self.driver1_respond = Driver_respond()
		
		rospy.Subscriber("/driver2_respond", Driver_respond, self.callback_driver2)
		self.driver2_respond = Driver_respond()
		self.timeStampe_driver = rospy.Time.now()

		rospy.Subscriber("/raw_vel", TwistWithCovarianceStamped, self.callback_rawvel)
		self.raw_vel = TwistWithCovarianceStamped()

		self.pub_taskDriver = rospy.Publisher("/task_driver", Int16, queue_size= 20)
		self.task_driver = Int16()
		# -- task driver
		self.taskDriver_nothing = 0
		self.taskDriver_resetRead = 1
		self.taskDriver_Read = 2
		self.task_driver.data = self.taskDriver_Read

		# -
		self.pub_disableBrake = rospy.Publisher("/disable_brake", Bool, queue_size= 20)
		self.disable_brake = Bool()

		# -- PS3 Joystick 
		self.pub_ps3 = rospy.Publisher("/control_ps3", Int8, queue_size=10)
		self.data_ps3 = Int8()

		# -- OdomNav request
		self.pub_OdomNavRequest = rospy.Publisher("/odom_nav_request", Int8, queue_size=10)
		self.data_OdomNavRequest = Int8()

		# -- Odom Respond
		rospy.Subscriber("/odom", Odometry, self.callback_Odometry)
		self.data_odometry = Odometry()

		# -- Nuc info
		rospy.Subscriber("/nuc_info", Nuc_info, self.callback_NucInfo)
		self.data_NucInfo = Nuc_info()

		# debug
		# self.pub_debug = rospy.Publisher("/debug_control", Debug_control, queue_size=100)
		# self.debug_control = Debug_control()	
		# -- HZ	
		self.FrequencePubBoard = 10.
		self.pre_timeBoard = 0
		# -- Mode operate
		self.mode_by_hand = 1
		self.mode_auto = 2
		self.mode_operate = self.mode_by_hand    # Lưu chế độ hoạt động.

		# -- Target
		self.target_x = 0.		 # lưu tọa độ điểm đích hiện tại.			
		self.target_y = 0.
		self.target_z = 0.
		self.target_tag = 0.
		self.target_listInfoPath  = []

		# -- Target Now
		self.cmd_target_x = 0.				
		self.cmd_target_y = 0.
		self.cmd_target_z = 0.
		self.cmd_target_tag = 0.
		self.cmd_listInfoPath = []

		self.process = -1        # tiến trình đang xử lý
		self.before_mission = 0  # nhiệm vụ trước khi di chuyển.
		self.after_mission = 0   # nhiệm vụ sau khi di chuyển.

		self.completed_before_mission = 0	 # Báo nhiệm vụ trước đã hoàn thành.
		self.completed_after_mission = 0	 # Báo nhiệm vụ sau đã hoàn thành.
		self.completed_move = 0			 	 # Báo di chuyển đã hoàn thành.
		self.completed_moveSimple = 0      # bao da den dich.
		self.completed_moveSpecial = 0     # bao da den aruco.
		self.completed_reset = 0             # hoan thanh reset.
		self.completed_MissionSetpose = 0 	#
		self.completed_checkLift = 0 	# kiem tra ke co hay ko sau khi nang. 
		# -- Flag
		self.flag_afterChager = 0
		self.flag_checkLiftError = 0 # cờ báo ko có kệ khi nâng. 
		self.flag_Auto_to_Byhand = 0
		self.flag_pause = 0
		self.enb_move = 0                    # cho phep navi di chuyen
		self.flag_read_client = 0
		self.flag_error = 0
		self.flag_warning = 0
		self.move_req = Move_request()
		self.pre_mess = ""               # lưu tin nhắn hiện tại.

		# -- Check:
		self.is_get_pose = 0
		self.is_mission = 0
		self.is_read_prepheral = 0       # ps2 - sti_read
		self.is_request_client = 0
		self.is_set_pose = 0
		self.is_zone_lidar = 0

		self.completed_wakeup = 0        # khi bật nguồn báo 1 sau khi setpose xong.
		# -- Status to server:
		self.statusU300L = 0
		self.statusU300L_ok = 0
		self.statusU300L_warning = 1
		self.statusU300L_error = 2	
		self.statusU300L_cancelMission = 5	
		# -- Status to detail to follow:
		self.stf = 0
		self.stf_wakeup = 0
		self.stf_running_simple = 1
		self.stf_stop_obstacle = 2
		self.stf_running_speial = 3
		self.stf_running_backward = 4
		self.stf_performUp = 5
		self.stf_performDown = 6
		# -- EMC reset
		self.EMC_resetOn = 1
		self.EMC_resetOff = 0
		self.EMC_reset = self.EMC_resetOff
		# -- EMC write
		self.EMC_writeOn = 1
		self.EMC_writeOff = 0
		self.EMC_write = self.EMC_writeOff
		# -- Led
		self.led_effect = 0
		self.led_error = 1 			# 1
		self.led_simpleRun = 2 		# 2
		self.led_specialRun = 3 	# 3
		self.led_perform = 4 		# 4
		self.led_completed = 5  	# 5	
		self.led_stopBarrier = 6  	# 6

		self.LED_OFF = 0
		self.LED_MOVE = 1
		self.LED_LEFT = 2
		self.LED_RIGHT = 3
		self.LED_BARRIER = 5
		self.LED_WARN = 7
		self.LED_ERROR = 8
		self.LED_SYS = 7

		# -- Mission server
		self.statusTask_liftError = 64 # trang thái nâng kệ nhueng ko có kệ.
		self.serverMission_liftUp = 65 # 1 65
		self.serverMission_liftDown = 66 # 2 66
		self.serverMission_charger = 6
		self.serverMission_unknown = 0
		self.serverMission_liftDown_charger = 10
		# -- Lift task.
		self.liftTask = 0 
		self.liftUp = 2
		self.liftDown = 1
		self.liftStop = 0
		self.liftResetOn = 1
		self.liftResetOff = 0
		self.liftReset = self.liftStop
		self.flag_commandLift = 0
		self.liftTask_byHand = self.liftStop
		# -- Speaker
		self.speaker = 0
		self.speaker_requir = 0 # luu trang thai cua loa
		self.SPK_OFF = 0
		self.SPK_WAIT = 2
		self.SPK_MOVE = 3
		self.SPK_LEFT = 4
		self.SPK_RIGHT = 5
		self.SPK_BARRIER = 7
		self.SPK_WARN = 9
		self.SPK_ERROR = 10
		self.SPK_SYS = 11

		self.enb_spk = 1
		# -- Charger
		self.charger_on = 1
		self.charger_off = 0		
		self.charger_requir = self.charger_off
		self.charger_write = self.charger_requir
		self.charger_valueOrigin = 0.2
		# -- Voltage
		self.timeCheckVoltage_charger = 1800 # s => 30 minutes.
		self.timeCheckVoltage_normal = 60     # s
		self.pre_timeVoltage = 0   # s
		self.valueVoltage = 0
		self.step_readVoltage = 0
		# -- Cmd_vel
		self.vel_ps2 = Twist()       # gui toc do
		# -- ps2 
		self.time_ht = rospy.get_time()
		self.time_tr = rospy.get_time()
		self.rate_cmdvel = 10. 
		# -- Error Type
		self.error_move = 0
		self.error_perform = 0
		self.error_device = 0  # camera(1) - MC(2) - Main(3) - SC(4)

		self.numberError = 0
		self.lastTime_checkLift = 0.0
		# -- add new
		self.enb_debug = 0
		# --
		self.listError = []
		self.job_doing = 0
		# -- -- -- Su dung cho truong hop khi AGV chuyen Che do bang tay, bi keo ra khoi vi tri => AGV se chay lai.
		# -- Pose tai vi tri Ke, sac
		self.poseWait = Pose()
		self.distance_resetMission = 0.1
		self.flag_resetFramework = 0
		# --
		self.flag_stopMove_byHand = 0
		# --
		self.timeStampe_reflectors = rospy.Time.now()
		# -- add 23/12/2021:
		self.pose_parkingRuning = Pose()
		print ("launch")
		# -- add 27/12/2021
		self.cancelbackward_pose = Pose()
		self.cancelbackward_offset = 0.0

		# -- add 18/01/2022 : sua loi di lai cac diem cu khi mat ket noi server.
		self.list_id_unknown = [0, 0, 0, 0, 0]
		self.flag_listPoint_ok = 0
		
		# -- add 19/01/2022 : Check error lost server.
		self.name_card = "wlp3s0b1"
		self.address = "192.168.1.40" # "172.21.15.224"
		self.saveTime_checkServer = rospy.Time.now()
		self.saveStatus_server = 0
		# -- add 30/03/2022 : co bao loi qua tai dong co.
		self.flagError_overLoad = 0
		# -- add 15/04/2022
		self.flag_listPointEmpty = 0

		# --
		self.step_tryTarget = 0

		# --
		self.updateTarget = 0

		# -- 
		self.saveTime_detectMapMathError = rospy.Time.now()
		self.saveTime_detectLocError = rospy.Time.now()

		self.nextProcessByTime = False
		self.timeWaitStart = -1
		self.saveTime_waitToStart  = rospy.Time.now()

		# -- add save pose map trusly
		self.savePose = Pose()
		self.needSetpose = False
		self.sttSetPose = 0 # 0: ko co | 1 : set roi
		self.isSlamOke = False
		self.flag_stopWhenMoving = 0
		self.saveOldDonextPlanClicked = 0

		self.flag_remote = 0

		self.flag_nextPlan = 0
		self.do_nextPlan = 0
		self.runOnce = 0

		# for debug
		self.step_log = 0
		self.ct_log = rospy.get_time()
		self.TIME_LOG = 1

		# for flag info
		self.flag_clearOutofPath = 0
		self.is_start = 0

		self.qt = 0

		self.step_clear_lidarloc = 0
		self.flag_clear_lidarloc = 0
		self.search_radius = 300
		self.number_clear_lidarloc = 0

		self.X = 0.0
		self.Y = 0.0
		self.Z = 0.0
		self.X_new = 0.0
		self.Y_new = 0.0
		self.Z_new = 0.0

		self.clear_lidarloc_time = 0

		self.step_setPose = 0

		# - 
		self.flaginfo_laserloc = 0
		self.theta_odom = 0.0
		self.theta_savepose = 0.0
		self.odom_start = Pose()
		self.is_firsttime_odomNav = False     # for not allow robot run odom sequently which odom run 2m then stop
		self.flag_byHand_to_Auto = 0
		self.is_errlls_matching = False
		self.is_errlls_loc = False
		self.allow_to_run = False

		self.savePose_Stop = Pose()
		self.savePoseStop_theta = 0.0
		self.is_firstime_savePoseStop = True
		self.is_firsttime_setPoseStop = True

	def callback_rawvel(self, data):
		self.raw_vel = data

	def callback_getPose(self, dat):
		self.robotPose_lidarLOC = dat
		# if self.lidarLoc_status.map_match_status == 90 and self.lidarLoc_status.loc_status == 10:
		# 	self.savePose = dat.pose

		# 	quaternion1 = (dat.pose.orientation.x, dat.pose.orientation.y,\
		# 					dat.pose.orientation.z, dat.pose.orientation.w)
		# 	euler = tf.transformations.euler_from_quaternion(quaternion1)

		# 	self.theta_savepose = round(euler[2], 3)

		# 	savePose_tuple = (self.savePose.position.x , self.savePose.position.y, self.theta_savepose)
		# 	self.pose_queue.append(savePose_tuple)
		# 	# self.pose_queue.append(self.theta_savepose)

		# 	print(self.pose_queue.display())

		# 	if self.completed_moveSimple == 1 and self.is_firstime_savePoseStop == True:
		# 		self.is_firstime_savePoseStop = False
		# 		self.savePose_Stop = dat.pose

		# 		q = (dat.pose.orientation.x, dat.pose.orientation.y,\
		# 						dat.pose.orientation.z, dat.pose.orientation.w)

		# 		e = tf.transformations.euler_from_quaternion(q)

		# 		self.savePoseStop_theta = round(e[2], 3)

		# doi quaternion -> rad
		quaternion1 = (dat.pose.orientation.x, dat.pose.orientation.y,\
					dat.pose.orientation.z, dat.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion1)

		self.NN_infoRespond.x = round(dat.pose.position.x, 3)
		self.NN_infoRespond.y = round(dat.pose.position.y, 3)
		self.NN_infoRespond.z = round(euler[2], 3)

	def lidarLoc_callback(self, data):
		self.lidarLoc_status = data

		if self.lidarLoc_status.map_match_status > 0 and self.lidarLoc_status.loc_status == 10:
			self.savePose = self.robotPose_lidarLOC.pose

			quaternion1 = (self.robotPose_lidarLOC.pose.orientation.x, self.robotPose_lidarLOC.pose.orientation.y,\
							self.robotPose_lidarLOC.pose.orientation.z, self.robotPose_lidarLOC.pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion1)

			self.theta_savepose = round(euler[2], 3)

			# -- add save pose to queue array
			# savePose_tuple = (self.savePose.position.x , self.savePose.position.y, self.theta_savepose)
			# self.pose_queue.append(savePose_tuple)
			# self.pose_queue.append(self.theta_savepose)
			# print(self.pose_queue.display())

			if self.completed_moveSimple == 1 and self.is_firstime_savePoseStop == True:
				self.is_firstime_savePoseStop = False
				self.savePose_Stop = dat.pose

				q = (self.robotPose_lidarLOC.pose.orientation.x, self.robotPose_lidarLOC.pose.orientation.y,\
								self.robotPose_lidarLOC.pose.orientation.z, self.robotPose_lidarLOC.pose.orientation.w)

				e = tf.transformations.euler_from_quaternion(q)

				self.savePoseStop_theta = round(e[2], 3)

	def nav350_callback(self, data):
		self.nav350_data = data

	def safety_NAV_callback(self, data):
		self.safety_NAV = data

	def callback_goalControl(self, data):
		self.status_goalControl = data
		self.timeStampe_statusGoalControl = rospy.Time.now()

	def callback_cancelMission(self, dat):
		self.cancelMission_control = dat

	def callback_infoRequest(self, dat):
		self.NN_infoRequest = dat
		self.timeStampe_server = rospy.Time.now()

	def callback_parking(self, dat):
		self.parking_status = dat
		self.is_parking_status = 1

	def callback_reconnect(self, dat):
		self.status_reconnect = dat

	def callback_main(self, dat):
		self.main_info = dat
		self.voltage = round(self.main_info.voltages, 2)
		self.timeStampe_main = rospy.Time.now()

	def callback_hc(self, dat):
		self.HC_info = dat
		self.timeStampe_HC = rospy.Time.now()

	def callback_toyoLeft(self, data):
		self.toyoLeft_info = data
		if self.toyoLeft_info.in1 == 1:
			self.saveData_toyoLeft = True
	
	def callback_toyoRight(self, data):
		self.toyoRight_info = data
		if self.toyoRight_info.in1 == 1:
			self.saveData_toyoRight = True

	# def callback_oc(self, dat):
	#     self.lift_status = dat
	#     self.timeStampe_OC = rospy.Time.now()

	def callback_appButton(self, dat):
		self.app_button = dat

	def callback_cmdRequest(self, dat):
		self.NN_cmdRequest = dat
		self.timeStampe_server = rospy.Time.now()
		# -- add 18/01/2022
		self.flag_listPoint_ok = 0

	# def callback_infoRequest(self, dat):
	# 	self.NN_infoRequest = dat

	def callback_safetyZone(self, dat):
		self.zoneRobot = dat
		self.is_readZone = 1
		self.timeStampe_safetyZone = rospy.Time.now()

	def callback_port(self, dat):
		self.status_port = dat

	def callback_driver1(self, dat):
		self.driver1_respond = dat
		self.timeStampe_driver = rospy.Time.now()

	def callback_driver2(self, dat):
		self.driver2_respond = dat
		self.timeStampe_driver = rospy.Time.now()

	def callback_NucInfo(self, dat):
		self.data_NucInfo = dat

	def callback_Odometry(self, dat):
		self.data_Odometry = dat
		quaternion1 = (dat.pose.pose.orientation.x, dat.pose.pose.orientation.y,\
						dat.pose.pose.orientation.z, dat.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion1)
		self.theta_odom = round(euler[2], 3)

	def callServiceInitializeAtPose(self, x, y, theta, sRadius):
		try:
			serClient = rospy.ServiceProxy('/LocInitializeAtPose', LocInitializeAtPoseSrv)
			# rospy.loginfo("Generated [], sending LocInitializeAtPose request...")
			resp = serClient(x, y, theta, sRadius)
			return resp.success

		except rospy.ServiceException as e:
			rospy.logwarn(e)
		
		return False

	def callServiceSetPose(self):
		data = PoseWithCovarianceStamped()

		data.header.frame_id = 'odom'
		data.pose.pose.position.x = self.savePose.position.x
		data.pose.pose.position.y = self.savePose.position.y
		data.pose.pose.orientation.z = self.savePose.orientation.z
		data.pose.pose.orientation.w = self.savePose.orientation.w


		try:
			srvSetpose = rospy.ServiceProxy('/set_pose', SetPose)
			resp = srvSetpose(data)
		except rospy.ServiceException as e:
			rospy.logwarn(e)

	# -- add 15/04/2022
	def check_listPoints(self, list_id):
		count = 0
		try: 
			for i in range(5):
				if (list_id[i] != 0):
					count += 1

			if count != 0:
				return 1
			else: 
				return 0
		except:
			return 0

	def log_mess(self, typ, mess, val):
		# -- add new
		if (self.enb_debug):
			if self.pre_mess != mess:
				if typ == "info":
					rospy.loginfo (mess + ": %s", val)
				elif typ == "warn":
					rospy.logwarn (mess + ": %s", val)
				else:
					rospy.logerr (mess + ": %s", val)
			self.pre_mess = mess

	def send_mess(self, type, mess):
		if self.step_log == 0:
			self.ct_log = rospy.get_time()
			self.step_log = 1
			# rospy.loginfo(mess)
		else:
			if rospy.get_time() - self.ct_log >= self.TIME_LOG:
				if type == 1:
					rospy.loginfo(mess)
				elif type == 2:
					rospy.logwarn(mess)
				elif type == 3:
					rospy.logerr(mess)

				self.ct_log = rospy.get_time()
				self.step_log = 0

	def pub_move_req(self, ena, ls):
		if rospy.get_time() - self.saveTime_pubRequestMove > float(1/self.rate_pubReqMove) : # < 20hz 
			self.saveTime_pubRequestMove = rospy.get_time()

			req_move = LineRequestMove()
			req_move.enable = ena
			req_move.target_x = ls.target_x
			req_move.target_y = ls.target_y
			req_move.target_z = ls.target_z
			req_move.pathInfo = ls.pathInfo

			if ls.target_x == self.fakePl.QT[0].target_x and ls.target_y == self.fakePl.QT[0].target_y:
				req_move.mess = 'Trên lộ trình tới điểm dừng 1'
			elif ls.target_x == self.fakePl.QT[1].target_x and ls.target_y == self.fakePl.QT[1].target_y:
				req_move.mess = 'Trên lộ trình tới điểm dừng 2'
			elif ls.target_x == self.fakePl.QT[2].target_x and ls.target_y == self.fakePl.QT[2].target_y:
				req_move.mess = 'Trên lộ trình tới điểm dừng 3'
			elif ls.target_x == self.fakePl.QT[3].target_x and ls.target_y == self.fakePl.QT[3].target_y:
				req_move.mess = 'Trên lộ trình tới điểm dừng 4'
			elif ls.target_x == self.fakePl.QT[4].target_x and ls.target_y == self.fakePl.QT[4].target_y:
				req_move.mess = 'Trên lộ trình tới điểm dừng 5'
			elif ls.target_x == self.fakePl.QT[5].target_x and ls.target_y == self.fakePl.QT[5].target_y:
				req_move.mess = 'Trên lộ trình tới điểm dừng 6'
			else:
				req_move.mess = 'Chưa xác nhận lộ trình'
			self.pub_moveReq.publish(req_move)

	def pub_cmdVel(self, twist , rate , time):
		self.time_ht = time 
		# print self.time_ht - self.time_tr
		# print 1/float(rate)
		if self.time_ht - self.time_tr > float(1/rate) : # < 20hz 
			self.time_tr = self.time_ht
		
		self.pub_vel.publish(twist)
		# else :
			# rospy.logwarn("Hz /cmd_vel OVER !! - %f", 1/float(self.time_ht - self.time_tr) )  

	def Main_pub(self, charge, sound, EMC_write, EMC_reset, _led_type):
		# charge, sound_on, sound_type, EMC_write, EMC_reset, OFF_5v , OFF_22v, led_button1, led_button2, a_coefficient , b_coefficient 
		mai = POWER_request()
		mai.charge = charge
		if sound == 0:
			mai.sound_on = 0	
		else:
			mai.sound_on = 1
			mai.sound_type = sound
		mai.EMC_write = EMC_write
		mai.EMC_reset = EMC_reset
		mai.led_type = _led_type
		
		self.pub_requestMain.publish(mai)

	def point_same_point(self, x1, y1, z1, x2, y2, z2):
		# tọa độ
		x = x2 - x1
		y = y2 - y1
		d = math.sqrt(x*x + y*y)
		# góc 
		if z2*z1 >= 0:
			z = z2 - z1
		else:
			z = z2 + z1
		if d > 0.2 or abs(z) > 0.14:  # 20 cm - ~ 20*C
			return 1
		else:
			return 0

	def pub_park(self, modeRun, poseBefore, poseTarget, offset):
		park = Parking_request()
		park.modeRun = modeRun
		park.poseBefore = poseBefore
		park.poseTarget = poseTarget
		park.offset = offset

		self.pub_parking.publish(park)

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat

	def readbatteryVoltage(self): # 
		time_curr = rospy.get_time()
		delta_time = (time_curr - self.pre_timeVoltage)
		if self.charger_requir == self.charger_on:
			self.flag_afterChager = 1
			if self.step_readVoltage == 0:  # bat sac.
				self.charger_write = self.charger_on
				if (delta_time > self.timeCheckVoltage_charger):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 1

			elif self.step_readVoltage == 1: # tat sac va doi.
				self.charger_write = self.charger_off
				if (delta_time > self.timeCheckVoltage_normal*3):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 2

			elif self.step_readVoltage == 2: # do pin.	
				bat = round(self.main_info.voltages, 1)*10
				# print "charger --"
				if  bat > 255:
					self.valueVoltage = 255
				elif bat < 0:
					self.valueVoltage = 0
				else:
					self.valueVoltage = bat

				self.pre_timeVoltage = time_curr
				self.step_readVoltage = 3

			elif self.step_readVoltage == 3: # doi.
				if (delta_time > 2):
					self.pre_timeVoltage = time_curr
					self.step_readVoltage = 0

		elif self.charger_requir == self.charger_off:
			if self.flag_afterChager == 1:   # sau khi tat sac doi T s roi moi do dien ap.
				self.pre_timeVoltage = time_curr
				self.flag_afterChager = 0
				self.charger_write = self.charger_off
				self.step_readVoltage = 0
			else:
				if (delta_time > self.timeCheckVoltage_normal):
					self.pre_timeVoltage = time_curr
					bat = round(self.main_info.voltages, 1)*10
					# print "normal --"
					if  bat > 255:
						self.valueVoltage = 255
					elif bat < 0:
						self.valueVoltage = 0
					else:
						self.valueVoltage = int(bat)

	def run_manual(self, vs_speed):
		cmd_vel = Twist()
		val_linear = (vs_speed/100.)*0.6           # 0.35
		val_rotate = (vs_speed/100.)*0.4            # 0.3

		if (self.app_button.bt_moveHand == 1):
			if (self.zoneRobot.zone_ahead == 1):
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.0
			else:
				cmd_vel.linear.x = val_linear
				cmd_vel.angular.z = 0.0

		if (self.app_button.bt_moveHand == 2):
			if (self.zoneRobot.zone_behind == 1):
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.0
			else:
				cmd_vel.linear.x = -val_linear
				cmd_vel.angular.z = 0.0

		if (self.app_button.bt_moveHand == 3):
			if (self.zoneRobot.zone_ahead == 1 or self.zoneRobot.zone_behind == 1):
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.0				
			else:
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = val_rotate

		if (self.app_button.bt_moveHand == 4):
			if (self.zoneRobot.zone_ahead == 1 or self.zoneRobot.zone_behind == 1):
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = 0.0				
			else:
				cmd_vel.linear.x = 0.0
				cmd_vel.angular.z = -val_rotate

		if (self.app_button.bt_moveHand == 0):
			cmd_vel.linear.x = 0.0
			cmd_vel.angular.z = 0.0

		if (self.safety_NAV.data == 1):
			cmd_vel = Twist()

		return cmd_vel

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def getPose_from_offset(self, pose_in, offset):
		pose_out = Pose()
		angle = self.quaternion_to_euler(pose_in.orientation)

		if (angle >= 0):
			angle_target = angle - pi
		else:
			angle_target = pi + angle

		pose_out.position.x = pose_in.position.x + cos(angle_target)*offset
		pose_out.position.y = pose_in.position.y + sin(angle_target)*offset

		pose_out.orientation = self.euler_to_quaternion(angle_target)
		return pose_out

	def detectLost_goalControl(self):
		delta_t = rospy.Time.now() - self.timeStampe_statusGoalControl
		if (delta_t.to_sec() > 1.4):
			return 1
		return 0

	def detectLost_driver(self):
		delta_t = rospy.Time.now() - self.timeStampe_driver
		if (delta_t.to_sec() > 0.8):
			return 1
		return 0

	def detectLost_hc(self):
		delta_t = rospy.Time.now() - self.timeStampe_HC
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_safetyZone(self):
		delta_t = rospy.Time.now() - self.timeStampe_safetyZone
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_oc(self):
		delta_t = rospy.Time.now() - self.timeStampe_OC
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_main(self):
		delta_t = rospy.Time.now() - self.timeStampe_main
		if (delta_t.to_sec() > 1.0):
			return 1
		return 0

	def detectLost_nav(self):
		delta_t = rospy.Time.now() - self.nav350_data.header.stamp
		if (delta_t.to_sec() > 0.4):
			return 1
		return 0

	def detectLost_lidarLOC(self):
		delta_t = rospy.Time.now() - self.lidarLoc_status.header.stamp
		if (delta_t.to_sec() > 1.):
			return 1
		return 0

	def detectLost_poseRobot(self):
		delta_t = rospy.Time.now() - self.robotPose_lidarLOC.header.stamp
		if (delta_t.to_sec() > 0.5):
			return 1
		return 0

	# -- add 19/01/2020
	def get_ipAuto(self, name_card): # name_card : str()
		try:
			address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
			# print ("address: ", address)
			return 1
		except Exception:
			return 0
			
	def check_server(self):
		is_ip = self.get_ipAuto(self.name_card)
		# is_ip = 1
		# time_ping = self.pingServer(self.address)
		time_ping = 0
		if (is_ip == 1):
			if (time_ping == -1):
				return 1 # khong Ping dc server
			else:
				return 0 # oki
		else:
			return 2 # khong lay dc IP

	# -- add 19/01/2022 : Check error lost server.
	# def detectLost_server(self):
	# 	delta_t = rospy.Time.now() - self.timeStampe_server
	# 	if (delta_t.to_sec() > 15):
	# 		return 1
	# 	return 0


	def detectLost_server(self):
		delta_t = rospy.Time.now() - self.timeStampe_server
		if (delta_t.to_sec() > 15):
			delta_s = rospy.Time.now() - self.saveTime_checkServer
			if (delta_s.to_sec() > 5):
				self.saveTime_checkServer = rospy.Time.now()
				self.saveStatus_server = self.check_server()

			if (self.saveStatus_server == 1):
				return 2
			elif (self.saveStatus_server == 2):
				return 3
			return 1
		return 0

	def detectLost_reflectors(self):
		# -- so luong guong
		if (self.nav350_data.number_reflectors >= 3): # loi mat guong
			self.timeStampe_reflectors = rospy.Time.now()

		delta_t = rospy.Time.now() - self.timeStampe_reflectors
		if (delta_t.to_sec() > 1.2):
			return 1
		return 0

	def checkStatus_mapMatchLoc(self):
		if self.lidarLoc_status.map_match_status > 0: # 30 
			self.saveTime_detectMapMathError = rospy.Time.now()
			
		delta_t = rospy.Time.now() - self.saveTime_detectMapMathError
		if (delta_t.to_sec() > 1):
			self.isSlamOke = False
			return 1
		return 0

	def checkStatus_Loc(self):
		if self.lidarLoc_status.loc_status < 20: 
			self.saveTime_detectLocError = rospy.Time.now()

		delta_t = rospy.Time.now() - self.saveTime_detectLocError
		if (delta_t.to_sec() > 1):
			self.isSlamOke = False
			return 1
		return 0


	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)

	def calculate_angle(self, qua1, qua2): # p1, p2 |
		euler1 = self.quaternion_to_euler(qua1)
		euler2 = self.quaternion_to_euler(qua2)

		delta_angle = euler2 - euler1
		if (abs(delta_angle) >= pi):
			if (delta_angle >= 0):
				delta_angle = (pi*2 - abs(delta_angle))*(-1)
			else:
				delta_angle = pi*2 - abs(delta_angle)
		return delta_angle

	def find_element(self, value_find, list_in):
		lenght = len(list_in)
		for i in range(lenght):
			if (value_find == list_in[i]):
				return 1
		return 0

	def synthetic_error(self):
		listError_now = []
		self.isSlamOke = True

		# -- Goal Control
		if self.detectLost_goalControl() == 1:
			listError_now.append(282)

		# -- EMG
		if self.main_info.EMC_status == 1:
			listError_now.append(121)

		# -- Va cham
		if self.HC_info.vacham == 1:
			listError_now.append(122)

		# -- lost HC
		if (self.detectLost_hc() == 1):
			listError_now.append(351)

		# -- lost CAN HC
		if (self.HC_info.status == -1):
			listError_now.append(352)

		# -- lost safety zone
		if (self.detectLost_safetyZone()):
			listError_now.append(240)

		# -- OC: ket noi                           # bỏ qua 
		# if (self.detectLost_oc() == 1):
		#     listError_now.append(341)

		# -- OC-CAN                                # bỏ qua
		# if (self.lift_status.status.data == -2):
		#     listError_now.append(343)

		# -- MAIN: CAN ket noi 
		if (self.main_info.CAN_status == 0):
			listError_now.append(323)

		#-- lost Main
		if (self.detectLost_main() == 1):
		    listError_now.append(321)

		#-- lost LidarLoc
		if (self.detectLost_lidarLOC() == 1):
			listError_now.append(221)

		#-- lost pose robot
		if (self.detectLost_poseRobot() == 1):
			listError_now.append(222)

		#-- Lost Driver 1
		if self.detectLost_driver() == 1: 
			listError_now.append(251)

		# if self.flag_clear_lidarloc == 0:
		# 	#-- LidarLoc: Matching error!
		# 	if (self.checkStatus_mapMatchLoc() == 1):
		# 		listError_now.append(241)

		# 	# -- LidarLoc: !
		# 	if (self.checkStatus_Loc() == 1):
		# 		listError_now.append(242)
		# else:
		# 	listError_now.append(454)

		# #-- LidarLoc: Matching error!
		# if (self.checkStatus_mapMatchLoc() == 1):
		# 	listError_now.append(241)

		# # -- LidarLoc: !
		# if (self.checkStatus_Loc() == 1):
		# 	listError_now.append(242)

		if self.flaginfo_laserloc == 0:
			#-- LidarLoc: Matching error!
			if (self.checkStatus_mapMatchLoc() == 1):
				listError_now.append(241)

			# -- LidarLoc: !
			if (self.checkStatus_Loc() == 1):
				listError_now.append(242)
		else:
			listError_now.append(1)

		#-- Error Driver 1
		summation1 = self.driver1_respond.alarm_all + self.driver1_respond.alarm_overload + self.driver1_respond.warning
		if (summation1 != 0):
			listError_now.append(252)

		#-- Lost Driver 2
		if self.detectLost_driver() == 1: 
			listError_now.append(261)

		#-- Error Driver 2
		summation2 = self.driver2_respond.alarm_all + self.driver2_respond.alarm_overload + self.driver2_respond.warning
		if (summation2 != 0):
			listError_now.append(262)

		# -- Loi mat nguong
		# if self.detectLost_reflectors() == 1: # loi mat guong
		# 	listError_now.append(272)

		# -- Ban Nang khong bat duoc cam bien
		# if (self.lift_status.status.data == 238 or self.lift_status.status.data == -1):
		#     listError_now.append(141)

		# -- Loi Code Parking:
		# if (self.parking_status.warning == 2):
		#     listError_now.append(281)

		# -- Nâng kệ nhưng không có kệ.
		# if self.flag_checkLiftError == 1:
		#     listError_now.append(471)

		# -- 19/01/2022 - Mat giao tiep voi Server 
		# sts_sr = self.detectLost_server()
		# if sts_sr == 1: # lost server
		#     listError_now.append(431)
		# elif sts_sr == 2: # lost server: Ping
		#     listError_now.append(432)
		# elif sts_sr == 3: # lost server: IP- wifi
		#     listError_now.append(433)

		# --- Low battery
		if self.voltage < 22:
			listError_now.append(451)

		# -- Co vat can khi di chuyen giua cac diem
		if (self.status_goalControl.safety == 1):
			listError_now.append(411)

		# -- Lỗi hệ thống
		if self.status_goalControl.error == 2:
			listError_now.append(292)

		# -- Co vat can khi di chuyen parking
		if (self.parking_status.warning == 1):
			listError_now.append(412)

		# -- AGV dung do da di het danh sach diem.
		if self.status_goalControl.misson == 1 or self.status_goalControl.misson == 3:
			if self.status_goalControl.complete_misson == 2:
				listError_now.append(441)
				# -- add 18/01/2022
				self.flag_listPoint_ok = 1

		# -- add 15/04/2022 - Danh sach ID lenh trong. Dung tranh AGV khac
		if (self.flag_listPointEmpty == 1):
			listError_now.append(442)

		# -- add 31/10/2024 - CPU > 99%
		if (self.data_NucInfo.cpu_usage >= 99):
			listError_now.append(443)
		
		if (self.flag_clearOutofPath == 1):
			listError_now.append(293)
		else:
			# -- Lỗi ra khởi đường dẫn
			if self.status_goalControl.error == 1:
				listError_now.append(291)

		return listError_now

	def resetAll_variable(self):
		self.enb_move = 0

		# if self.parking_status.status == 11:  # khi doi lenh. no reset truoc khi parking nhan ra no da hoan thanh.
		# 	self.flag_requirBackward = 1

		self.enb_parking = 0

		self.completed_before_mission = 0
		self.completed_after_mission = 0
		self.completed_move = 0
		self.completed_moveSimple = 0
		self.completed_moveSpecial = 0
		self.completed_backward = 0
		self.completed_MissionSetpose = 0

		self.completed_checkLift = 0
		self.flag_checkLiftError = 0

		self.enb_mission = 0
		self.mission = 0
		self.lifttable = 0
		self.conveyor = 0

		# -- add 18/01/2022
		self.flag_listPoint_ok = 0

		# rospy.logwarn("Update new target from: X= %s | Y= %s to X= %s| Y= %s", self.target_x, self.target_y, self.NN_cmdRequest.target_x, self.NN_cmdRequest.target_y)
		print("info", "Update new target: X_new = ", self.cmd_target_x)
		print("info", "Update new target: Y_new = ", self.cmd_target_y)
		self.target_x = self.cmd_target_x
		self.target_y = self.cmd_target_y
		self.target_listInfoPath = self.cmd_listInfoPath
		# self.target_z = self.NN_cmdRequest.target_z
		# self.target_tag = self.NN_cmdRequest.tag
		# self.before_mission = self.NN_cmdRequest.before_mission
		# self.after_mission = self.NN_cmdRequest.after_mission
		# -- add 12/11/2021
		self.flag_resetFramework = 0
		self.flag_Auto_to_Byhand = 0

		# -- add 30/03/2022 : co bao loi qua tai dong co.
		# self.flagError_overLoad = 0
		# -- add 15/04/2022
		self.flag_listPointEmpty = 0
		self.flag_stopWhenMoving = 0

	def moving_shape(seld, linear_x, angular_z):
			R = 30.
			if linear_x == 0.0 and angular_z == 0.0:
				return 0 # dừng
			elif fabs(angular_z) < 0.002 and linear_x > 0.:
				return 1 # đi thẳng
			elif fabs(angular_z) > 0.0 and linear_x == 0:
				return 2 # quay tại chỗ
			else:
				try:
					r = linear_x/angular_z
					if abs(r) < R:
						if r > 0:
							return 3 # rẽ trái
						else:
							return 4 # rẽ phải
						
				except ZeroDivisionError:
					print("Lỗi chia cho 0")
					
				return 1

	def update_QT(self):
		if self.status_goalControl.misson == 0:								
			self.qt = self.fakePl.agvAtQT(self.NN_infoRespond.x, self.NN_infoRespond.y)
			print ("qt va qtnow: ", self.qt, self.qtNow)
			self.qtNow = self.qt

		if self.numQt == 0:
			print("khong co quy trinh!")

		else:
			print("qt now", self.qtNow)
			self.cmd_target_x = self.fakePl.QT[self.qtNow].target_x
			self.cmd_target_y = self.fakePl.QT[self.qtNow].target_y
			self.cmd_listInfoPath = self.fakePl.QT[self.qtNow].pathInfo

	def update_QT_XY(self, X, Y):
		if self.status_goalControl.misson == 0:								
			self.qt = self.fakePl.agvAtQT(X, Y )
			print ("qt va qtnow: ", self.qt, self.qtNow)
			self.qtNow = self.qt

		if self.numQt == 0:
			print("khong co quy trinh!")

		else:
			print("qt now", self.qtNow)
			self.cmd_target_x = self.fakePl.QT[self.qtNow].target_x
			self.cmd_target_y = self.fakePl.QT[self.qtNow].target_y
			self.cmd_listInfoPath = self.fakePl.QT[self.qtNow].pathInfo

	def run(self):
		if self.process == -1: # khi moi khoi dong len
			# mode_operate = mode_hand
			time.sleep(0.2)
			self.mode_operate = self.mode_by_hand
			self.led = 0
			self.speaker_requir = self.SPK_WARN
			self.process = 0
			self.enb_parking = 0

		elif self.process == 0:	# chờ cac node khoi dong xong.
			ct = 8
			if ct == 8:
				self.process = 1

		elif self.process == 1: # reset toan bo: Main - MC - OC.
			self.flag_error = 0
			self.completed_before_mission = 0
			self.completed_after_mission = 0
			self.completed_move = 0
			self.completed_checkLift = 0
			self.liftTask = 0
			self.error_device = 0
			self.error_move = 0
			self.error_perform = 0
			self.process = 2

		elif self.process == 2: # kiem tra toan bo thiet bi ok.
			self.listError =  self.synthetic_error()
			self.numberError = len(self.listError)
			lenght = len(self.listError)
			
			count_error = 0
			count_warning = 0

			for i in range(lenght):
				if (self.listError[i] < 400 and self.listError[i] > 100):
					count_error += 1
				elif self.listError[i] >= 400:
					count_warning += 1
					
			# -- add 30/03/2022 : co bao loi qua tai dong co.
			if count_error == 0 and count_warning == 0: # and self.flagError_overLoad == 0:
				self.flag_error = 0
				self.flag_warning = 0

			elif count_error == 0 and count_warning > 0:
				self.flag_error = 0
				self.flag_warning = 1

			else:
				self.flag_error = 1
				self.flag_warning = 0

			if (self.flag_error == 1):
				self.statusU300L = self.statusU300L_error
			else:
				if (self.flag_warning == 1):
					self.statusU300L = self.statusU300L_warning
				else:
					self.statusU300L = self.statusU300L_ok

			# -- ERROR
			if self.app_button.bt_clearError == 1 or self.main_info.stsButton_reset == 1:
				if self.flag_requirResetparking == 1:
					self.enb_parking = 0
					self.flag_requirResetparking = 0

				# -- dung parking neu co loi. - thay doi
				# self.enb_parking = 2

				# self.EMC_write = self.EMC_writeOff
				self.EMC_reset = self.EMC_resetOn
				# -- sent clear error
				self.flag_error = 0
				self.error_device = 0
				self.error_move = 0
				self.error_perform = 0

				self.flag_checkLiftError = 0
				self.task_driver.data = self.taskDriver_resetRead

				# -- add 30/03/2022 : co bao loi qua tai dong co.
				self.flagError_overLoad = 0

				# -- Xóa lỗi từ gói Goal Control
				if self.find_element(292, self.listError) == 1: 
					self.enb_move = 0
				
				if self.find_element(291, self.listError) == 1:
					self.flag_clearOutofPath = 1
					self.enb_move = 0

				# if self.clear_lidarloc_time == 1:   # Mởi khởi động 
				# 	if self.find_element(241, self.listError) == 1 or self.find_element(242, self.listError) == 1:
				# 		self.step_clear_lidarloc = 1
				# 		self.flag_clear_lidarloc = 1
				# 		self.clear_lidarloc_time = 2

				# -- add 23/7/2024
				if (self.find_element(241, self.listError) == 0 and self.find_element(242, self.listError) == 0) and (self.find_element(221, self.listError) == 0 and self.find_element(222, self.listError) == 0):
					self.update_QT()

			else:
				self.task_driver.data = self.taskDriver_Read
				self.EMC_reset = self.EMC_resetOff

				# -- Add new: 23/12: Khi mat ket Driver, EMG duoc keo len.
				# if (self.flag_error == 1):
				# 	if (self.find_element(251, self.listError) == 1 or self.find_element(261, self.listError) == 1):
				# 		self.EMC_write = self.EMC_writeOn
				# 	else:
				# 		self.EMC_write = self.EMC_writeOff
				

			# if (self.error_device != 0 or self.error_perform != 0 or self.error_move != 0):
			# 	self.flag_error = 1	

			self.process = 3

		elif self.process == 3: # read app
			if self.app_button.bt_passHand == 1:
				if self.mode_operate == self.mode_auto: # keo co bao dang o tu dong -> chuyen sang bang tay.
					self.flag_Auto_to_Byhand = 1
					self.completed_moveSimple = 0

				self.mode_operate = self.mode_by_hand

			if self.app_button.bt_passAuto == 1:
				self.mode_operate = self.mode_auto
				print("Giá trị của biến flag_nextplan là: ", self.flag_nextPlan)

			if self.mode_operate == self.mode_by_hand:
				self.process = 30
				if self.app_button.ck_remote == 1:
					self.flag_remote = 1
					self.data_ps3.data = 1
				else:
					self.flag_remote = 0
					self.data_ps3.data = 0

			elif self.mode_operate == self.mode_auto:
				self.process = 40
				self.disable_brake.data = 0
				self.flag_remote = 0
	# ------------------------------------------------------------------------------------
	# -- BY HAND:
		elif self.process == 30:
			self.enb_move = 0
			self.job_doing = 20

			# -
			self.flag_clearOutofPath = 0

			if self.parking_status.status != 0:
				self.enb_parking = 0

			# ------------------------------------------------------------
			if self.flag_remote == 0:
				if self.flag_error == 0:
				# -- Send vel
					# if self.lift_status.status.data == 0 or self.lift_status.status.data >= 3:  # Đang thực hiện nhiệm vụ ở chế độ auto -> ko cho phép di chuyển.
						# -- Move
					self.pub_cmdVel(self.run_manual(self.app_button.vs_speed), self.rate_cmdvel, rospy.get_time())

					# else:
					#     self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

				else: # -- Has error
					if self.find_element(241, self.listError) == 1 or self.find_element(242, self.listError) == 1:
						self.pub_cmdVel(self.run_manual(self.app_button.vs_speed), self.rate_cmdvel, rospy.get_time())
					else:
						self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

			# ------------------------------------------------------------
			# -- Speaker
			if self.app_button.bt_speaker == True:
				self.enb_spk = 1
				if self.app_button.bt_setting == 1:
					if self.app_button.soundtype == 0:
						self.speaker_requir = self.SPK_OFF
					elif self.app_button.soundtype == 1:
						self.speaker_requir = self.SPK_WAIT
					elif self.app_button.soundtype == 2:
						self.speaker_requir = self.SPK_MOVE
					elif self.app_button.soundtype == 3:
						self.speaker_requir = self.SPK_LEFT
					elif self.app_button.soundtype == 4:
						self.speaker_requir = self.SPK_RIGHT
					elif self.app_button.soundtype == 5:
						self.speaker_requir = self.SPK_BARRIER
					elif self.app_button.soundtype == 6:
						self.speaker_requir = self.SPK_WARN
					elif self.app_button.soundtype == 7:
						self.speaker_requir = self.SPK_ERROR
					elif self.app_button.soundtype == 8:
						self.speaker_requir = self.SPK_SYS
				else:
					self.speaker_requir = self.SPK_WAIT

			else:
				self.enb_spk = 0
				self.speaker_requir = self.SPK_OFF

			# led
			if self.app_button.bt_setting == 1:
				if self.app_button.ledtype == 0:
					self.led_effect = self.LED_OFF
				elif self.app_button.ledtype == 1:
					self.led_effect = self.LED_MOVE
				elif self.app_button.ledtype == 2:
					self.led_effect = self.LED_LEFT
				elif self.app_button.ledtype == 3:
					self.led_effect = self.LED_RIGHT
				elif self.app_button.ledtype == 4:
					self.led_effect = self.LED_BARRIER
				elif self.app_button.ledtype == 5:
					self.led_effect = self.LED_WARN
				elif self.app_button.ledtype == 6:
					self.led_effect = self.LED_ERROR
				elif self.app_button.ledtype == 7:
					self.led_effect = self.LED_SYS
			else:
				self.led_effect = self.LED_MOVE

			# -- Charger
			if self.app_button.bt_charger == True:
				self.charger_requir = self.charger_on
			else:
				self.charger_requir = self.charger_off

			# -- Keep shaft.
			self.disable_brake.data = self.app_button.bt_brake

			# -- reset status of auto mode in manual mode
			if self.flag_byHand_to_Auto == 1:
				self.flaginfo_laserloc = 0
				self.flag_byHand_to_Auto = 0

			# --
			#if self.app_button.bt_resetFrameWork == 1:
			#	self.resetAll_variable()

			self.process = 2

	# -- RUN AUTO:
		elif self.process == 40:    # AGV có đang lỗi ko ??
			self.flag_byHand_to_Auto = 1
			if self.flag_error == 1: 
				self.job_doing = 30
				self.enb_mission = 0
				self.process = 2

				# -- receive and save toyo signal from Machine Chain
				if self.saveData_toyoLeft == True or self.saveData_toyoRight == True: 
					self.allow_to_run = True
					self.saveData_toyoLeft = False
					self.saveData_toyoRight = False

				# -- action of AGV When AGV get special error!
				if self.find_element(291, self.listError) == 1 or self.find_element(293, self.listError) == 1 or self.find_element(241, self.listError) == 1 or self.find_element(242, self.listError) == 1:
					if self.find_element(291, self.listError) == 1:
						self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

					if self.find_element(293, self.listError) == 1:
						self.pub_cmdVel(self.run_manual(50), self.rate_cmdvel, rospy.get_time())

						if self.app_button.bt_doNextPlan2 == 1:
							self.flag_clearOutofPath = 0
							self.completed_moveSimple = 0
							self.flag_nextPlan = 1
							print("Đã hết lỗi path")
							self.update_QT()

					# - update 23/10/2024 - lidarloc err
					if (self.find_element(241, self.listError) == 1 or self.find_element(242, self.listError) == 1) and self.is_firsttime_odomNav == False:
						if self.is_start != 0:     # robot not in startup state
							if self.completed_moveSimple == 0:    # robot is still moving to target point
								# - confirm err
								if self.find_element(241, self.listError) == 1:
									self.is_errlls_matching = True
									self.is_errlls_loc = False

								elif self.find_element(242, self.listError) == 1:
									self.is_errlls_matching = False
									self.is_errlls_loc = True

								print("AGV chayj theo odom --------------------------------")
								# - reset goal_control
								# self.enb_move = 0
								# - pub vel = 0
								# self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

								# - reset odom data
								self.callServiceSetPose()

								# - Create range compare 2 odom and pos
								X_down = self.savePose.position.x - 0.1
								Y_down = self.savePose.position.y - 0.1
								Z_down = self.theta_savepose - 0.17

								X_up = self.savePose.position.x + 0.1
								Y_up = self.savePose.position.y + 0.1
								Z_up = self.theta_savepose + 0.17

								print("Khoang gioi han odom la: {x}, {y}, {t}, {z}, {w}, {v}".format(x = X_down, y = Y_down, t = Z_down, z = X_up, w = Y_up, v = Z_up))

								# - wait respond from node Odom
								if X_down < self.data_Odometry.pose.pose.position.x < X_up and Y_down < self.data_Odometry.pose.pose.position.y < Y_up and Z_down < self.theta_odom < Z_up:
									self.flaginfo_laserloc = 1
									self.data_OdomNavRequest.data = 1
									print("Da guii yeu cau agv chay theo odom")
									self.odom_start = self.data_Odometry.pose.pose
									self.is_firsttime_odomNav = True
							
							else:      # robot is in target point
								# - initial pose if lidaloc error
								if self.is_firsttime_setPoseStop == True:
									self.is_firsttime_setPoseStop = False
									print("Set pose for lls if lls error matching or localization")
									self.X = self.savePose_Stop.position.x
									self.Y = self.savePose_Stop.position.y
									self.Z = degrees(self.savePoseStop_theta)

									self.search_radius = 300
									
									while self.search_radius <= 2000:
										stt = self.callServiceInitializeAtPose(int(self.X*1000), int(self.Y*1000), int(self.Z*1000), self.search_radius)
										self.search_radius += 100

										if stt:
											break

								# - recv toyo when error
								if self.allow_to_run == True:
									print("AGV chayj theo odom --------------------------------")
									# - reset goal_control
									# self.enb_move = 0
									# - pub vel = 0
									# self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

									# - reset odom data
									self.callServiceSetPose()

									# - Create range compare 2 odom and pos
									X_down = self.savePose.position.x - 0.1
									Y_down = self.savePose.position.y - 0.1
									Z_down = self.theta_savepose - 0.17

									X_up = self.savePose.position.x + 0.1
									Y_up = self.savePose.position.y + 0.1
									Z_up = self.theta_savepose + 0.17

									print("Khoang gioi han odom la: {x}, {y}, {t}, {z}, {w}, {v}".format(x = X_down, y = Y_down, t = Z_down, z = X_up, w = Y_up, v = Z_up))

									# - wait respond from node Odom
									if X_down < self.data_Odometry.pose.pose.position.x < X_up and Y_down < self.data_Odometry.pose.pose.position.y < Y_up and Z_down < self.theta_odom < Z_up:
										self.flaginfo_laserloc = 1
										self.data_OdomNavRequest.data = 1
										print("Da guii yeu cau agv chay theo odom khi robot dang cho hang")
										self.odom_start = self.data_Odometry.pose.pose
										self.is_firsttime_odomNav = True
										# self.completed_moveSimple = 0
										self.allow_to_run = False

				else:
					self.enb_move = 0
					self.pub_cmdVel(Twist(), self.rate_cmdvel, rospy.get_time())

			else:
				# -- lidarloc ok
				self.process = 41
				if self.lidarLoc_status.map_match_status == 90 and self.lidarLoc_status.loc_status == 10:
					self.flaginfo_laserloc = 0
					self.data_OdomNavRequest.data = 0
					self.is_firsttime_odomNav = False
					self.is_firsttime_setPoseStop = True

				if self.flaginfo_laserloc == 1:
					print("AGV dang bam theo odom")
					# -- reset save pose for odom
					self.is_firsttime_savePoseStop = True

					d = self.calculate_distance(self.data_Odometry.pose.pose.position, self.odom_start.position)

					if d > 2:
						print("AGV chay qua duoc 1.5 m nhung chua phuc hoi loi matching-----------------------------")
						self.flaginfo_laserloc = 0 
						self.data_OdomNavRequest.data = 0
						self.enb_move = 0

						# - chooose pose for service in end of path
						self.X = self.data_Odometry.pose.pose.position.x
						self.Y = self.data_Odometry.pose.pose.position.y
						self.Z = degrees(self.theta_odom)
					
						# print("Vị trí AGV trước khi xảy ra lỗi là: {x}, {y}, {z}".format(x = self.X, y = self.Y, z = self.Z))
						# self.X_new = self.X + 20
						# self.Y_new = self.Y + 20
						# self.Z_new = self.Z + 90
					
						# if self.Z_new >= 360:
						# 	self.Z_new = self.Z_new - 360
						
						# -- call service setpose
						self.search_radius = 300

						# for i in range(0,3):
						# 	stt = self.callServiceInitializeAtPose(int(self.X_new*1000), int(self.Y_new*1000), int(self.Z_new*1000), 800)
						
						while self.search_radius <= 2000:
							stt = self.callServiceInitializeAtPose(int(self.X*1000), int(self.Y*1000), int(self.Z*1000), self.search_radius)
							self.search_radius += 100
							if stt:
								break

						# -- update QT anyway 
						self.update_QT_XY(self.data_Odometry.pose.pose.position.x, self.data_Odometry.pose.pose.position.y)
						self.timeWaitStart = -1
									
					if self.is_errlls_loc == False and self.is_errlls_matching == True:
						pass

					elif self.is_errlls_loc == True and self.is_errlls_matching == False:
						# -- call service setpose
						print("Set pose for AGV follow odom data")
		
		elif self.process == 41:    # Kiểm tra các điều kiện cho phép AGV sang lộ trình mới
			if self.flag_clearOutofPath == 0:
				## Kiểm tra xem nút có được nhấn ko??
				if self.app_button.bt_doNextPlan == 1 and self.runOnce == 0:
					self.runOnce = 1
					self.flag_nextPlan = self.flag_nextPlan + 1
						
					if self.flag_nextPlan > 2:
						self.flag_nextPlan = 1

					self.send_mess(1, "Nút next plan đã được nhấn = " + str(self.flag_nextPlan))

				elif self.app_button.bt_doNextPlan == 0:
					self.runOnce = 0

				# -- 
				if self.is_start == 0:      # khi lần đầu chuyển chế dộ tự động sau khởi động
					self.process = 2
					self.job_doing = 10
					self.send_mess(1, "AGV mới khởi động lên")
					if self.flag_nextPlan == 1:
						self.is_start = 1

				else:
					if self.completed_moveSimple == 0:    # Chưa tới điêm đích
						if self.flag_nextPlan == 0:
							self.process = 42

						elif self.flag_nextPlan == 1:
							if self.job_doing == 10:
								self.update_QT()               

							else:
								# - Tìm lại vị trí sau khi tạm dùng
								if self.flag_Auto_to_Byhand == 1:
									rospy.logwarn("AGV tiếp tục chạy sau khi chuyển lại chế độ tự động")
									self.flag_Auto_to_Byhand = 0
									self.update_QT()

								if self.flag_pause == 1:
									rospy.logwarn("AGV tiếp tục chạy sau tạm dừng")
									self.flag_pause = 0
									self.update_QT()			

							self.process = 42

						elif self.flag_nextPlan == 2:
							# self.flag_Auto_to_Byhand = 1
							self.flag_pause = 1
							self.enb_move = 0
							self.process = 2
							self.job_doing = 6
							self.send_mess(1, "AGV đang ở trạng thái tạm dừng")

					else:
						if self.do_nextPlan == 1:
							self.update_QT()
								
							self.timeWaitStart = -1
							self.do_nextPlan = 0

						self.process = 42
			else:
				self.process = 2
				print("AGV vẫn đang gặp lỗi")
				
		elif self.process == 42:                # update Leenhj
			if ( self.target_x != self.cmd_target_x ) or ( self.target_y != self.cmd_target_y):      # update lệnh
				if (self.cmd_target_x < 500.) and (self.cmd_target_y < 500.):
					self.resetAll_variable()
					self.process = 46
					self.send_mess(2, "Hệ thống reset !!!!!!!!!!")
					print("-------------------------------------")
					
				else:
					self.send_mess(1, "Have new target but Not fit: X= " + str(self.cmd_target_x))    ## Rase an error
					self.send_mess(1, "Have new target but Not fit: y= " + str(self.cmd_target_y))
					self.process = 2

				self.NN_infoRespond.offset = 0 # --
			
			else:
				self.process = 46

		elif self.process == 46:	# Thuc hien di chuyen diem thuong.
			if self.completed_moveSimple == 1:      #
				# self.cmd_t5arget_x= 40.
					
				self.process = 35
				self.enb_move = 0
			else:
				self.job_doing = 5
				# -- reset save pose for odom
				self.is_firsttime_savePoseStop = True

				# self.send_mess(1, "Chiều dài của list lệnh là: " + str(len(self.target_listInfoPath)))
				# if (len(self.NN_cmdRequest.list_x) != 0) and (len(self.NN_cmdRequest.list_y) != 0) and (self.target_x < 500) and (self.target_y < 500):
				if len(self.target_listInfoPath) != 0:
					self.request_move.target_x = self.target_x
					self.request_move.target_y = self.target_y
					self.request_move.target_z = self.target_z

					self.request_move.pathInfo = self.target_listInfoPath

					# if self.before_mission == self.serverMission_liftUp or self.before_mission == 1: # Nâng
					# 	self.move_req.mission = 1
					# else:
					# 	self.move_req.mission = 0

					self.enb_move = 1 # -- vung To

					# -- add 19/01/2022 : chuyen vung sick.
					# if self.before_mission == self.serverMission_liftUp or self.before_mission == 1: # Nâng
					# 	self.enb_move = 1 # -- vung To
					# else:
					# 	self.enb_move = 3 # -- vung Nho
				# else:
				# 	self.log_mess("warn", "ERROR: Target of List point wrong !!!", 0)

				# -- add 19/01/2022 : chuyen vung sick.
				# if self.status_goalControl.misson == 1 or self.status_goalControl.misson == 3:
				if self.status_goalControl.complete_misson == 1 and self.status_goalControl.target_x == self.request_move.target_x and self.status_goalControl.target_y == self.request_move.target_y and self.status_goalControl.target_z == self.request_move.target_z:  # Hoan thanh di chuyen.
					self.completed_moveSimple = 1
					self.flag_nextPlan = 0

					print("recieve data done :", self.request_move.target_x, self.request_move.target_y, self.request_move.target_z)
					
					self.timeWaitStart = self.fakePl.needWaitTimeToNextProcess(self.qtNow)
					if self.timeWaitStart != -1:
						self.saveTime_waitToStart  = rospy.Time.now()
					
					print("Thòi gian đợi tự khởi động là: ", self.timeWaitStart) 

					self.qtNow = self.qtNow + 1
					if self.qtNow >= self.numQt:
						self.qtNow = 0

					self.enb_move = 0
					print("Move completed")

				self.process = 2	

		elif self.process == 35:
			self.job_doing = 8
			self.process = 2

			if self.timeWaitStart != -1:
				delta = rospy.Time.now() - self.saveTime_waitToStart
				if delta.to_sec() > self.timeWaitStart:
					self.flag_nextPlan = 1
					self.timeWaitStart = -1
					self.send_mess(1, "Đáp ứng thời gian, AGV tự chạy")
					# self.do_nextPlan = 1
					# self.job_doing = 5
					# self.send_mess(2, "AGV xác nhận tiếp tục chạy")

			if self.toyoLeft_info.in1 == 1 or self.toyoRight_info.in1 == 1:
				self.flag_nextPlan = 1
				self.send_mess(1, "có tín hiệu toyo")
				# self.do_nextPlan = 1
				# self.job_doing = 5
				# self.send_mess(2, "AGV xác nhận tiếp tục chạy")

			if self.allow_to_run == True:
				self.allow_to_run = False
				self.flag_nextPlan = 1
				self.do_nextPlan = 1
				self.job_doing = 5
				self.send_mess(2, "AGV xác nhận tiếp tục chạy khi co tin hieu toyo")

			if self.flag_nextPlan == 1:
				self.do_nextPlan = 1
				self.job_doing = 5
				self.send_mess(2, "AGV xác nhận tiếp tục chạy")

			self.send_mess(1,"Wating new Target ...")

		# if self.mode_operate == self.mode_auto:
		# 	if self.isSlamOke == False:
		# 		self.step_setPose = 1
		# 	else:
		# 		self.sttSetPose = 0
		
		#if self.step_setPose == 1 and self.sttSetPose == 0:
		#	self.X = self.savePose.position.x
		#	self.Y = self.savePose.position.y
		#	euler = euler_from_quaternion((0,0, self.savePose.orientation.z,  self.savePose.orientation.w))
		#	self.Z = degrees(euler[2])
		#
		#	print("Vị trí AGV trước khi xảy ra lỗi là: {x}, {y}, {z}".format(x = self.X, y = self.Y, z = self.Z))
		#	self.X_new = self.X + 20
		#	self.Y_new = self.Y + 20
		#	self.Z_new = self.Z + 90
		#
		#	if self.Z_new >= 360:
		#		self.Z_new = self.Z_new - 360
		#
			# self.step_setPose = 2
		#	self.search_radius = 300

		#	for i in range(0,3):
		#		stt = self.callServiceInitializeAtPose(int(self.X_new*1000), int(self.Y_new*1000), int(self.Z_new*1000), 800)
		#	
		#	while self.search_radius <= 2000:
		#		stt = self.callServiceInitializeAtPose(int(self.X*1000), int(self.Y*1000), int(self.Z*1000), self.search_radius)
		#		self.search_radius += 100
		#		if stt:
		#			break
					# self.step_setPose = 0
					# print("Vị trí AGV sau khi lấy lại là: {x}, {y}".format(x = self.NN_infoRespond.x, y = self.NN_infoRespond.Y))
					# if (self.X - 0.1) < self.NN_infoRespond.x < (self.X + 0.1) and (self.Y - 0.1) < self.NN_infoRespond.Y < (self.Y + 0.1):
					# 	break
					# else:
					# 	self.flag_RBerrPos = 1
					# 	break	
		#	
		#	self.step_setPose = 0
		#	self.sttSetPose = 1

		
		# -- Các bước fix lỗi mất matching
		# if self.step_clear_lidarloc == 0:
		# 	if self.statusU300L == self.statusU300L_error:
		# 		self.step_clear_lidarloc = -1

		# elif self.step_clear_lidarloc == -1:
		# 	if self.statusU300L == self.statusU300L_ok:
		# 		self.clear_lidarloc_time = 1

		# elif self.step_clear_lidarloc == 1:    # những setup ban đầu
		# 	self.disable_brake.data = 1      # Mở phanh ra
		# 	self.X = self.savePose.position.x
		# 	self.Y = self.savePose.position.y
		# 	euler = euler_from_quaternion((0,0, self.savePose.orientation.z,  self.savePose.orientation.w))
		# 	self.Z = degrees(euler[2])

		# 	print("Vị trí AGV trước khi xảy ra lỗi là: {x}, {y}, {z}".format(x = self.X, y = self.Y, z = self.Z))
		# 	self.X_new = self.X + 20
		# 	self.Y_new = self.Y + 20
		# 	self.Z_new = self.Z + 90

		# 	if self.Z_new >= 360:
		# 		self.Z_new = self.Z_new - 360

		# 	self.step_clear_lidarloc = 2
		# 	self.search_radius = 300
		
		# elif self.step_clear_lidarloc == 2:   # Kéo AGV ra hẵn vị trí pose khác
			
		# 	stt = self.callServiceInitializeAtPose(int(self.X_new*1000), int(self.Y_new*1000), int(self.Z_new*1000), 800)
		# 	self.number_clear_lidarloc += 1
		# 	if self.number_clear_lidarloc > 2:
		# 		self.step_clear_lidarloc = 3
		# 		self.number_clear_lidarloc = 0

		# elif self.step_clear_lidarloc == 3:   # Kéo lại vị trí AGV từ vị trí lúc bị mất
		# 	if self.search_radius <= 2000:
		# 		stt = self.callServiceInitializeAtPose(int(self.X*1000), int(self.Y*1000), int(self.Z*1000), self.search_radius)
				
		# 		if stt:
		# 			self.step_clear_lidarloc = 4
		# 		else:
		# 			self.search_radius += 100
		# 	else:
		# 		self.step_clear_lidarloc = 4
		
		# elif self.step_clear_lidarloc == 4:   # Hoàn thành
		# 	self.step_clear_lidarloc = 0
		# 	self.flag_clear_lidarloc = 0
		# 	self.clear_lidarloc_time = 1

		# -- tu dong set pose
		# if self.mode_operate == self.mode_auto:
		# 	if self.isSlamOke:
		# 		self.sttSetPose = 0
		# 	else:
		# 		if self.sttSetPose == 0 and self.zoneRobot.zone_ahead == 0:
		# 			euler = euler_from_quaternion((0,0, self.savePose.orientation.z,  self.savePose.orientation.w))
		# 			dG = int(degrees(euler[2])*1000)
		# 			for i in range(0,5):
		# 				stt = self.callServiceInitializeAtPose(int(self.savePose.position.x*1000), int(self.savePose.position.y*1000), dG, 800)
		# 				if stt:
		# 					print("set pose thanh cong, lan: ", i)
		# 					self.sttSetPose = 1
		# 					break

		# 				print("set pose false")
		# 			self.sttSetPose = 1	

		# -- Tag + Offset:
		if self.mode_operate == self.mode_auto:
			if self.completed_move == 1:
				self.NN_infoRespond.tag = self.NN_cmdRequest.tag
				self.NN_infoRespond.offset = self.NN_cmdRequest.offset
			else:
				self.NN_infoRespond.tag = 0
				self.NN_infoRespond.offset = 0

			if self.completed_before_mission == 1 and self.completed_after_mission == 0 and self.flag_checkLiftError == 0:
				self.NN_infoRespond.task_status = self.before_mission
			elif self.completed_before_mission == 1 and self.completed_after_mission == 0 and self.flag_checkLiftError == 1:
				self.NN_infoRespond.task_status = self.statusTask_liftError
			if self.completed_before_mission == 1 and self.completed_after_mission == 1:
				self.NN_infoRespond.task_status = self.after_mission

		self.NN_infoRespond.status = self.statusU300L    # Status: Error
		self.NN_infoRespond.error_perform = self.process
		self.NN_infoRespond.error_moving = self.flag_error
		self.NN_infoRespond.error_device = self.numberError
		self.NN_infoRespond.listError = self.listError
		self.NN_infoRespond.process = self.job_doing

		# -- Battery - ok
		self.readbatteryVoltage()
		self.NN_infoRespond.battery = int(self.valueVoltage)
		
		if self.flag_cancelMission == 0:
			# -- mode respond server
			if self.mode_operate == self.mode_by_hand:         # Che do by Hand
				self.NN_infoRespond.mode = 1

			elif self.mode_operate == self.mode_auto:          # Che do Auto
				self.NN_infoRespond.mode = 2
		else:
			self.NN_infoRespond.mode = 5

		# -- Respond Client
		self.pub_infoRespond.publish(self.NN_infoRespond)    # Pub Client

		# -- Request Navigation
		self.pub_move_req(self.enb_move, self.request_move)  # Pub Navigation

		# -- Odom nav request
		self.pub_OdomNavRequest.publish(self.data_OdomNavRequest)

		# -- Speaker && led
		if self.app_button.bt_setting == 0:
			if self.mode_operate == self.mode_auto:
				# speaker
				if self.flag_error == 1 and self.flag_warning == 1:
					self.speaker_requir = self.SPK_ERROR
					self.led_effect = self.LED_ERROR

				elif self.flag_error == 1 and self.flag_warning == 0:
					if self.find_element(292, self.listError) == 1 or self.find_element(291, self.listError) == 1:
						self.speaker_requir = self.SPK_SYS
						self.led_effect = self.LED_SYS
					else:
						self.speaker_requir = self.SPK_ERROR
						self.led_effect = self.LED_ERROR

				elif self.flag_error == 0 and self.flag_warning == 1:
					if self.find_element(411, self.listError) == 1:
						self.speaker_requir = self.SPK_BARRIER
						self.led_effect = self.LED_BARRIER
					else:
						self.speaker_requir = self.SPK_WARN
						self.led_effect = self.LED_WARN			
				else:
					k = self.moving_shape(self.raw_vel.twist.twist.linear.x, self.raw_vel.twist.twist.angular.z)

					if k == 0:
						self.speaker_requir = self.SPK_MOVE
						self.led_effect = self.LED_MOVE
					elif k == 1:
						self.speaker_requir = self.SPK_MOVE
						self.led_effect = self.LED_MOVE
					elif k == 2:
						self.speaker_requir = self.SPK_MOVE
						self.led_effect = self.LED_MOVE
					elif k == 3:
						self.speaker_requir = self.SPK_MOVE
						self.led_effect = self.LED_LEFT
					elif k == 4:
						self.speaker_requir = self.SPK_MOVE
						self.led_effect = self.LED_RIGHT

			elif self.mode_operate == self.mode_by_hand:
				self.speaker_requir = self.SPK_OFF
				self.led_effect = self.LED_MOVE					

		# -- -- -- pub Board
		time_curr = rospy.get_time()
		d = (time_curr - self.pre_timeBoard)
		if (d > float(1/self.FrequencePubBoard)): # < 20hz 
			self.pre_timeBoard = time_curr
			# -- Request OC:
			# self.lift_control.control.data = self.liftTask
			# self.lift_control.reset.data = self.liftReset
			# self.pub_OC.publish(self.lift_control)

			# -- Request Main: charger, sound, EMC_write, EMC_reset
			if self.enb_spk == 1:
				# tat Loa khi sac thanh cong!
				if self.charger_write == self.charger_on and self.main_info.charge_current >= self.charger_valueOrigin and self.flag_error == 0:
					self.speaker = self.SPK_OFF
				else:
					self.speaker = self.speaker_requir
			else:
				self.speaker = self.SPK_OFF

			self.Main_pub(self.charger_write, self.speaker, self.EMC_write, self.EMC_reset, self.led_effect)  # MISSION

			# -- Request HC:
			self.HC_request.RBG1 = self.led_effect 
			self.HC_request.RBG2 = self.led_effect
			self.pub_HC.publish(self.HC_request)

			# -- Request task Driver:
			self.pub_taskDriver.publish(self.task_driver)

			# ---------------- Brake ---------------- #
			self.pub_disableBrake.publish(self.disable_brake)

			# ------------ PS3 -----------------------
			self.pub_ps3.publish(self.data_ps3)
			
		# -- cancel mission
		if self.cancelMission_control.data == 1:
			self.flag_cancelMission = 1
			self.cancelMission_status.data = 1

		if self.NN_cmdRequest.id_command == 0:
			self.flag_cancelMission = 0
			self.cancelMission_status.data = 0

		self.pub_cancelMission.publish(self.cancelMission_status)
		
		self.pub_park(self.enb_parking, self.parking_poseBefore, self.parking_poseTarget, self.parking_offset)
		self.rate.sleep()

def main():
	# Start the job threads
	class_1 = ros_control()
	# Keep the main thread running, otherwise signals are ignored.
	while not rospy.is_shutdown():
		class_1.run()

if __name__ == '__main__':
	main()

"""
Stt :
0: chờ đủ dữ liệu để parking
1: chờ tín hiệu parking
21:  tính khoảng cách tiến lùi
-21: thực hiện di chuyen tiến lùi
-210: thực hiện quay trước nếu gặp TH AGV bị lệch góc lớn
31: tính góc quay để lùi vào kệ
-31: thực hiện quay
41: Parking
51: completed - Đợi Reset
52: error: bien doi tf loi

"""
