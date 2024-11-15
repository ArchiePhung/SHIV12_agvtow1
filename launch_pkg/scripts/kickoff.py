#!/usr/bin/env python3
# Author: HOANG VAN QUANG - BEE
# DATE: 22/06/2021

from sensor_msgs.msg import LaserScan, Image

from sensor_msgs.msg import Imu as Imu_ss

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sti_msgs.msg import *
from std_msgs.msg import Int16, Int8
from message_pkg.msg import *

from sick_lidar_localization.msg import LocalizationControllerResultMessage0502
from sti_msgs.msg import *
from message_pkg.msg import *

import roslaunch
import rospy
import string
import time
import os

class Launch:
    def __init__(self, file_launch):
        # -- parameter
        self.fileLaunch = file_launch
        # -- launch
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        # -- variable
        self.process = 0
        self.time_pre = time.time()

    def start(self):
        if (self.process == 0): # - Launch
            # print ("Launch node!")
            launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.fileLaunch])
            launch.start()
            self.process = 1  

    def start_and_wait(self, timeWait): # second - Dung cho cac node ko pub.
        if (self.process == 0): # - Launch
            # print ("Launch node!")
            launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.fileLaunch])
            launch.start()
            self.process = 1
            self.time_pre = time.time()
            return 0

        elif (self.process == 1): # - Wait
            t = (time.time() - self.time_pre)%60
            if (t > timeWait):
                self.process = 2
            return 0

        elif (self.process == 2): # - Wait
            return 1

class scanMap():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('kickoff_node', anonymous=False)
		self.rate = rospy.Rate(10)

		self.count_node = 0.0
		self.notification = ''
		self.step = 0
		self.timeWait = 0.4 # s

		self.pub_stausLaunch = rospy.Publisher('status_launch', Status_launch, queue_size= 10)
		self.stausLaunch = Status_launch()

		# - 0 - module - firstWork.
		self.path_firstWork = rospy.get_param('path_firstWork', '')
		self.launch_firstWork = Launch(self.path_firstWork)
		rospy.Subscriber('/first_work/run', Int16, self.callBack_firstWork)
		self.is_firstWork = 0
		self.count_node += 1

		# - 1 - module - checkPort.
		self.path_checkPort = rospy.get_param('path_checkPort', '')
		self.launch_checkPort = Launch(self.path_checkPort)
		rospy.Subscriber('/status_port', Status_port, self.callBack_checkPort)
		self.is_checkPort = 0
		self.count_node += 1

		# - 2 - module - reconnectAll.
		self.path_reconnectAll = rospy.get_param('path_reconnectAll', '')
		self.launch_reconnectAll = Launch(self.path_reconnectAll)
		rospy.Subscriber('/status_reconnect', Status_reconnect, self.callBack_reconnectAll)
		self.is_reconnectAll = 0
		self.count_node += 1

		# - 3 - module - Board - CAN.
		self.path_board = rospy.get_param('path_board', '')
		self.launch_board = Launch(self.path_board)
		rospy.Subscriber('/CAN_received', CAN_received, self.callBack_board)
		self.is_board = 0
		self.count_node += 1

		# - 3 - module - Board - CAN.
		self.path_controlLight = rospy.get_param('path_controlLight', '')
		self.launch_controlLight = Launch(self.path_controlLight)
		rospy.Subscriber('/led_reply', Int8, self.callBack_controlLight)
		self.is_light = 0
		self.count_node += 1

		# - 3.5 - module - MC - encoder.
		self.path_mc = rospy.get_param('path_mc', '')
		self.launch_mc = Launch(self.path_mc)
		rospy.Subscriber('/encoder_respond', EncoderData, self.callBack_mc)
		self.is_mc = 0
		self.count_node += 1

		# - 4 - module - Board - convert CAN.
		self.path_convertCAN = rospy.get_param('path_convertCAN', '')
		self.launch_convertCAN = Launch(self.path_convertCAN)
		self.count_node += 1

		# - 5 - module - magLine
		self.path_magLine = rospy.get_param('path_magLine', '')
		self.launch_magLine = Launch(self.path_magLine)
		rospy.Subscriber('/magneticLine1', Magnetic_line, self.callBack_magLine)
		self.is_magLine = 0
		self.count_node += 1

		# # - 6 - module - Driver Left
		# self.path_driverLeft = rospy.get_param('path_driverLeft', '')
		# self.launch_driverLeft = Launch(self.path_driverLeft)
		# rospy.Subscriber('/driver1_respond', Driver_respond, self.callBack_driverLeft)
		# self.is_driverLeft = 0
		# self.count_node += 1

		# # - 7 - module - driverRight.
		# self.path_driverRight = rospy.get_param('path_driverRight', '')
		# self.launch_driverRight = Launch(self.path_driverRight)
		# rospy.Subscriber('/driver2_respond', Driver_respond, self.callBack_driverRight)
		# self.is_driverRight = 0
		# self.count_node += 1

		# - driver all
		self.path_drivers = rospy.get_param('path_drivers', '')
		self.launch_drivers = Launch(self.path_drivers)
		rospy.Subscriber('/driver1_respond', Driver_respond, self.callBack_driverLeft)
		self.is_drivers = 0
		self.count_node += 1

		# - 8 - module - kinematic.
		self.path_kinematic = rospy.get_param('path_kinematic', '')
		self.launch_kinematic = Launch(self.path_kinematic)
		rospy.Subscriber('/driver1_query', Driver_query, self.callBack_driverRequest)
		self.is_kinematic = 0
		self.count_node += 1

		# - 9 - module - tf Full.
		self.path_tfFull = rospy.get_param('path_tfFull', '')
		self.launch_tfFull = Launch(self.path_tfFull)
		self.is_tfFull = 1
		self.count_node += 1

		# - 10 - module - lidar.
		self.path_lidarFull = rospy.get_param('path_lidarFull', '')
		self.launch_lidarFull = Launch(self.path_lidarFull)
		rospy.Subscriber('/sick_safetyscanners/scan', LaserScan, self.callBack_lidarFull)
		self.is_lidarFull = 0
		self.count_node += 1

		# - 11 - module - safetyZone.
		self.path_safetyZone = rospy.get_param('path_safetyZone', '')
		self.launch_safetyZone = Launch(self.path_safetyZone)
		rospy.Subscriber('/safety_zone', Zone_lidar_2head, self.callBack_safetyZone)
		self.is_safetyZone = 0
		self.count_node += 1

		# -- module - lidar.
		self.path_lidarLOC = rospy.get_param('path_lidarLOC', '')
		self.launch_lidarLOC = Launch(self.path_lidarLOC)
		rospy.Subscriber('/localizationcontroller/out/localizationcontroller_result_message_0502', LocalizationControllerResultMessage0502, self.callBack_lidarLOC)
		self.is_lidarLOC = 0
		self.count_node += 1

		# -- odom

		# - 12 - module - IMU. raw_imu_bno055
		self.path_imu = rospy.get_param('path_imu', '')
		self.launch_imu = Launch(self.path_imu)
		rospy.Subscriber('/imu/data', Imu_ss, self.callBack_imu)
		self.is_imu = 0
		self.count_node += 1

		# - 13 - module - imuFilter.
		self.path_imuFilter = rospy.get_param('path_imuFilter', '')
		self.launch_imuFilter = Launch(self.path_imuFilter)
		rospy.Subscriber('/imu/data_bno055', Imu_ss, self.callBack_imuFilter)
		self.is_imuFilter = 0
		self.count_node += 1

		# - 14 - module odomEncoder.
		self.path_odomEncoder = rospy.get_param('path_odomEncoder', '')
		self.launch_odomEncoder = Launch(self.path_odomEncoder)
		rospy.Subscriber('/raw_odom', Odometry, self.callBack_odomEncoder)
		self.is_odomEncoder = 0
		self.count_node += 1

		# - 15 - module - path_odomHector.
		self.path_odomHector = rospy.get_param('path_odomHector', '')
		self.launch_odomHector = Launch(self.path_odomHector)
		rospy.Subscriber('/scanmatch_odom', Odometry, self.callBack_odomHector)
		self.is_odomHector = 0
		self.count_node += 1

		# - 16 - module - Camera.
		self.path_camera = rospy.get_param('path_camera', '')
		self.launch_camera = Launch(self.path_camera)
		rospy.Subscriber('/camera/color/image_raw', Image, self.callBack_camera)
		self.is_camera = 0
		self.count_node += 1

		# - 17 - module - poseLidar.
		self.path_poseLidar = rospy.get_param('path_poseLidar', '')
		self.launch_poseLidar = Launch(self.path_poseLidar)
		rospy.Subscriber('/pose_lidar', PoseWithCovarianceStamped, self.callBack_poseLidar)
		self.is_poseLidar = 0
		self.count_node += 1

		# - 18 - module - rf2o odom.
		self.path_rf2oOdom = rospy.get_param('path_rf2oOdom', '')
		print ("self.path_rf2oOdom: ", self.path_rf2oOdom)
		self.launch_rf2oOdom = Launch(self.path_rf2oOdom)
		rospy.Subscriber('/odom_lms100', Odometry, self.callBack_rf2oOdom)
		self.is_rf2oOdom = 0
		self.count_node += 1

		# - 19 - module - mapServer.
		self.path_mapServer = rospy.get_param('path_mapServer', '')
		self.launch_mapServer = Launch(self.path_mapServer)
		rospy.Subscriber('/map', OccupancyGrid, self.callBack_mapServer)
		self.is_mapServer = 0
		self.count_node += 1

		# - 20 - module - ekf.
		self.path_ekf = rospy.get_param('path_ekf', '')
		self.launch_ekf = Launch(self.path_ekf)
		rospy.Subscriber('/odom', Odometry, self.callBack_ekf)
		self.is_ekf = 0
		self.count_node += 1

		# - 21 - module - amcl.
		self.path_amcl = rospy.get_param('path_amcl', '')
		self.launch_amcl = Launch(self.path_amcl)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callBack_amcl)
		self.is_amcl = 0
		self.count_node += 1

		# - 22 - module - posePublisher.
		self.path_posePublisher = rospy.get_param('path_posePublisher', '')
		self.launch_posePublisher = Launch(self.path_posePublisher)
		rospy.Subscriber('/robotPose_lidarLOC', PoseStamped, self.callBack_posePublisher)
		self.is_posePublisher = 0
		self.count_node += 1

		# - 23 - module - Setpose.
		self.path_setPose = rospy.get_param('path_setPose', '')
		self.launch_setPose = Launch(self.path_setPose)
		rospy.Subscriber('/setpose_status', Setpose_status, self.callBack_setPose)
		self.is_setPose = 0
		self.count_node += 1

		# - 24 - module - Parking.
		self.path_parking = rospy.get_param('path_parking', '')
		self.launch_parking = Launch(self.path_parking)
		rospy.Subscriber('/parking_respond', Parking_respond, self.callBack_parking)
		self.is_parking = 0
		self.count_node += 1

		# - 25 - launch_detectTag
		self.path_detectTag = rospy.get_param('path_detectTag', '')
		self.launch_detectTag = Launch(self.path_detectTag)
		self.is_detectTag = 0
		self.count_node += 1

		# - 26 - Atuo launch file detect
		self.path_apriltag = rospy.get_param('path_apriltag', '')
		self.launch_apriltag = Launch(self.path_apriltag)
		# rospy.Subscriber('/parking_status', Parking_status, self.callBack_apriltag)
		self.is_apriltag = 0
		self.count_node += 1

		# - 27 - module - navigation.
		self.path_navigation = rospy.get_param('path_navigation', '')
		self.launch_navigation = Launch(self.path_navigation)
		rospy.Subscriber('/move_respond', Status_goal_control, self.callBack_navigation)
		self.is_navigation = 0
		self.count_node += 1

		# - 28 - module - stiControl.
		self.path_stiControl = rospy.get_param('path_stiControl', '')
		self.launch_stiControl = Launch(self.path_stiControl)
		rospy.Subscriber('/NN_infoRespond', NN_infoRespond, self.callBack_stiControl)
		self.is_stiControl = 0
		self.count_node += 1

		# - 29 - module - client.
		self.path_client = rospy.get_param('path_client', '')
		self.launch_client = Launch(self.path_client)
		rospy.Subscriber('/NN_infoRequest', NN_infoRequest, self.callBack_client)
		self.is_client = 0
		self.count_node += 1

		# - 29 - module - client.
		self.path_client = rospy.get_param('path_client', '')
		self.launch_client = Launch(self.path_client)
		rospy.Subscriber('/NN_infoRequest', NN_infoRequest, self.callBack_client)
		self.is_client = 0
		self.count_node += 1

		# -- module - cartographer.
		self.path_sound = rospy.get_param('path_sound', '')
		self.launch_soundRequest = Launch(self.path_sound)
		# rospy.Subscriber('/', Int16, self.callBack_cartographer)
		self.is_sound = 1
		self.count_node += 1

		# - module - nuc info.
		self.path_nucInfo = rospy.get_param('path_nucInfo', '')
		self.launch_nucInfo = Launch(self.path_nucInfo)
		rospy.Subscriber("/nuc_info", Nuc_info, self.callback_nuc)
		self.is_nucInfo = 0
		self.count_node += 1

		# -- module odom
		self.path_odomAll = rospy.get_param('path_odomAll', '')
		self.launch_odomAll = Launch(self.path_odomAll)
		# rospy.Subscriber("/odom", Nuc_info, self.callback_odom)
		self.is_odomAll = 0
		self.count_node += 1

	# - 0 -
	def callBack_firstWork(self, data):
		self.is_firstWork = 1
	# - 1 -
	def callBack_checkPort(self, data):
		self.is_checkPort = 1
	# - 2 -
	def callBack_reconnectAll(self, data):
		self.is_reconnectAll = 1
	# - 3 -
	def callBack_board(self, data):
		self.is_board = 1
	# - 3.5 -
	def callBack_mc(self, data):
		self.is_mc = 1
	# - 4 -
	def callBack_controlLight(self, data):
		self.is_light = 1
	# - 5 -
	def callBack_magLine(self, data):
		self.is_magLine = 1
	# - 6 -
	def callBack_driverLeft(self, data):
		self.is_drivers = 1
	# - 7 -
	def callBack_driverRight(self, data):
		self.is_driverRight = 1
	# - 8 -
	def callBack_driverRequest(self, data):
		self.is_kinematic = 1
	# - 9 -
	# - 10 -
	def callBack_lidarFull(self, data):
		self.is_lidarFull = 1
	# - 11 -
	def callBack_safetyZone(self, data):
		self.is_safetyZone = 1
	# - 12 -
	def callBack_imu(self, data):
		self.is_imu = 1
	# - 13 -
	def callBack_imuFilter(self, data):
		self.is_imuFilter = 1
	# - 14 -
	def callBack_odomEncoder(self, data):
		self.is_odomEncoder = 1
	# - 15 -
	def callBack_odomHector(self, data):
		self.is_odomHector = 1
	# - 16 -
	def callBack_camera(self, data):
		self.is_camera = 1
	# - 17 -
	def callBack_poseLidar(self, data):
		self.is_poseLidar = 1
	# - 18-
	def callBack_rf2oOdom(self, data):
		self.is_rf2oOdom = 1
	# - 19-
	def callBack_mapServer(self, data):
		self.is_mapServer = 1
	# - 20 -
	def callBack_amcl(self, data):
		self.is_amcl = 1
	# - 21 -
	def callBack_ekf(self, data):
		self.is_ekf = 1
	# - 22 -
	def callBack_posePublisher(self, data):
		self.is_posePublisher = 1
	# - 23 -
	def callBack_setPose(self, data):
		self.is_setPose = 1
	# - 24 -
	def callBack_parking(self, data):
		self.is_parking = 1
	# - 25 -
	def callBack_navigation(self, data):
		self.is_navigation = 1
	# - 26 -
	def callBack_stiControl(self, data):
		self.is_stiControl = 1
	# - 27 - 
	def callBack_client(self, data):
		self.is_client = 1
	# - 28 -
	def callBack_cartographer(self, data):
		self.is_cartographer = 1
	# - -
	def callBack_lidarLOC(self, data):
		self.is_lidarLOC = 1

	def callback_nuc(self, data):
		self.is_nucInfo = 1

	def run(self):
		while not rospy.is_shutdown():
			# - 0 - firstWork
			if (self.step == 0): 
				self.notification = 'launch_firstWork'
				self.launch_firstWork.start()
				if (self.is_firstWork == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- checkPort
			elif (self.step == 1):
				self.notification = 'launch_checkPort'
				self.launch_checkPort.start()
				if (self.is_checkPort == 1):
					self.step += 1
					time.sleep(self.timeWait)

			elif (self.step == 2):
				self.notification = 'launch_reconnectAll'
				self.launch_reconnectAll.start()
				self.is_reconnectAll = 1
				if (self.is_reconnectAll == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- Board - CAN.
			elif (self.step == 3):
				self.notification = 'launch_board'
				self.launch_board.start()
				if (self.is_board == 1):
					self.step += 1
					# self.step = 5
					time.sleep(self.timeWait)

			# # -- Board - CAN.
			# elif (self.step == 4):
			# 	self.notification = 'launch_controlLight'
			# 	self.launch_controlLight.start()
			# 	if (self.is_light == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# -- MC - board
			elif (self.step == 4):
				self.notification = 'launch_mc'
				self.launch_mc.start()
				if (self.is_mc == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- Board - convert CAN
			elif (self.step == 5):
				self.notification = 'launch_convertCAN'
				sts = self.launch_convertCAN.start_and_wait(2)
				if sts == 1:
					self.step += 1
					self.step = 6
					time.sleep(self.timeWait)

			# -- Magline
			# elif (self.step == 5):
			# 	self.notification = 'launch_magLine'
			# 	self.launch_magLine.start()
			# 	if self.is_magLine == 1:
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# -- Motor - Left
			# elif (self.step == 6):
			# 	self.notification = 'launch_driverLeft'
			# 	self.launch_driverLeft.start()
			# 	if (self.is_driverLeft == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait + 2)

			# -- Motor - Right
			# elif (self.step == 7):
			# 	self.notification = 'launch_driverRight'
			# 	self.launch_driverRight.start()
			# 	if (self.is_driverRight == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# -- Motor - All
			elif (self.step == 6):
				self.notification = 'launch_driverAll'
				sts = self.launch_drivers.start_and_wait(3.)
				if (self.is_drivers == 1 or sts == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- kinematic
			elif (self.step == 7):
				self.notification = 'launch_kinematic'
				self.launch_kinematic.start()
				if (self.is_kinematic == 1):
					self.step += 1
					self.step = 10
					time.sleep(self.timeWait)

            # -- tf Full
			# elif (self.step == 9):
			# 	self.notification = 'launch_tfFull'
			# 	self.launch_tfFull.start()
			# 	if (self.is_tfFull == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

            # -- Lidar
			elif (self.step == 10):
				self.notification = 'launch_lidarFull'
				self.launch_lidarFull.start()
				if (self.is_lidarFull == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- safetyZone
			elif (self.step == 11):
				self.notification = 'launch_safetyZone'
				self.launch_safetyZone.start()
				if (self.is_safetyZone == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- lidarLOC
			elif (self.step == 12):
				self.notification = 'launch_lidarLOC'
				self.launch_lidarLOC.start()
				if (self.is_lidarLOC == 1):
					self.step += 1
					self.step = 18
					time.sleep(self.timeWait)

			# # -- IMU
			# elif (self.step == 12):
			# 	self.notification = 'launch_imu'
			# 	self.launch_imu.start()
			# 	if (self.is_imu == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# # -- imuFilter
			# elif (self.step == 13):
			# 	self.notification = 'launch_imuFilter'
			# 	self.launch_imuFilter.start()
			# 	if (self.is_imuFilter == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# # -- odomEncoder
			# elif (self.step == 14):
			# 	self.notification = 'launch_odomEncoder'
			# 	self.launch_odomEncoder.start()
			# 	if (self.is_odomEncoder == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# # -- odomHector
			# elif (self.step == 15):
			# 	self.is_odomHector = 1
			# 	self.notification = 'launch_odomHector'
			# 	# self.launch_odomHector.start()
			# 	if (self.is_odomHector == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# # -- Camera
			# elif (self.step == 16):
			# 	self.notification = 'launch_camera'
			# 	self.launch_camera.start()
			# 	if (self.is_camera == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# # -- poseLidar
			# elif (self.step == 17):
			# 	self.is_poseLidar = 1
			# 	self.notification = 'launch_poseLidar'
			# 	self.launch_poseLidar.start()
			# 	if (self.is_poseLidar == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# -- rf2o_odom
			elif (self.step == 18):
				self.notification = 'launch_odomAll'
				sts = self.launch_odomAll.start_and_wait(2.)
				if (self.is_odomAll == 1 or sts == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# # -- Ekf
			elif (self.step == 19):
				self.notification = 'launch_ekf'
				self.launch_ekf.start()
				if (self.is_ekf == 1):
					self.step += 1
					self.step = 22
					time.sleep(self.timeWait)

			# # -- Map server
			# elif (self.step == 20):
			# 	self.notification = 'launch_mapServer'
			# 	self.launch_mapServer.start()
			# 	if (self.is_mapServer == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			
			# # -- AMCL
			# elif (self.step == 21):
			# 	self.notification = 'launch_amcl'
			# 	self.launch_amcl.start()
			# 	if (self.is_amcl == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# -- Pose Publisher
			elif (self.step == 22):
				self.notification = 'launch_posePublisher'
				self.launch_posePublisher.start()
				if (self.is_posePublisher == 1):
					self.step = 26
					time.sleep(self.timeWait)

			# # -- Setpose
			# elif (self.step == 23):
			# 	self.notification = 'launch_setPose'
			# 	self.launch_setPose.start()
			# 	if (self.is_setPose == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# -- Parking
			elif (self.step == 24):
				self.notification = 'launch_parking'
				# self.is_parking = 1
				self.launch_parking.start()
				if (self.is_parking == 1):
					self.step = 26
					time.sleep(self.timeWait)

			# -- Detect Tag
			# elif (self.step == 25):
			# 	self.notification = 'launch_detectTag'
			# 	sts = self.launch_detectTag.start_and_wait(4.)
			# 	if (self.is_detectTag == 1 or sts == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# # -- Apriltag
			# elif (self.step == 25):
			# 	self.notification = 'launch_apriltag'
			# 	sts = self.launch_apriltag.start_and_wait(4.)
			# 	if (self.is_apriltag == 1 or sts == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# -- Navigation
			elif (self.step == 26):
				self.notification = 'launch_navigation'
				self.is_navigation = 1
				self.launch_navigation.start()
				if (self.is_navigation == 1):
					self.step += 1
					time.sleep(self.timeWait)
				
			# -- Sti Control
			elif (self.step == 27):
				self.notification = 'launch_stiControl'
				self.launch_stiControl.start()
				if (self.is_stiControl == 1):
					self.step += 1
					self.step = 28
					time.sleep(self.timeWait)
					
			# -- client
			# elif (self.step == 28):
			# 	self.notification = 'launch_client'
			# 	# self.step += 1
			# 	sts = self.launch_client.start_and_wait(3.)
			# 	if (self.is_client == 1 or sts == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# -- Nuc info
			elif (self.step == 28):
				self.notification = 'launch_nucInfo'
				# self.step += 1
				sts = self.launch_nucInfo.start_and_wait(3.)
				if (self.is_nucInfo == 1 or sts == 1):
					self.step += 1
					self.step = 29
					time.sleep(self.timeWait)


            # -- Completed
			elif (self.step == 29):
				# self.launch_soundRequest.start()
				self.notification = 'Completed!'
            
			self.count_node = 29
            # -- -- PUBLISH STATUS
			self.stausLaunch.persent = int(float(self.step/self.count_node)*100.)
			self.stausLaunch.position = self.step
			self.stausLaunch.notification = self.notification
			self.pub_stausLaunch.publish(self.stausLaunch)
			# time.sleep(0.1)
			self.rate.sleep()

def main():
	print('Program starting')
	try:
		program = scanMap()
		program.run()
	except rospy.ROSInterruptException:
		pass
	print('Programer stopped')

if __name__ == '__main__':
	main()
