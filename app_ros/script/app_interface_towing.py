#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Developer: Ho Phuc Hoang
Company: STI Viet Nam
Date: 5/1/2024 

"""

import sys
import time
import threading
import signal
import json

import random

import os
import re  
import subprocess
import argparse
from datetime import datetime

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPaintEvent
from PyQt5.QtWidgets import QWidget

sys.path.append('/home/stivietnam/catkin_ws/devel/lib/python3/dist-packages')
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

import roslib
import rospy

from sti_msgs.msg import *
from message_pkg.msg import *
from std_msgs.msg import Int16, Bool, Int8
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from sick_lidar_localization.srv import LocGetLocalizationStatusSrv, LocInitializeAtPoseSrv, LocAutoStartSavePoseSrv
from sick_lidar_localization.msg import LocalizationControllerResultMessage0502

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees
from enum import Enum

from PyQt5.uic import loadUi
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import * # QDialog, QApplication, QWidget
from PyQt5.QtGui import * # QPixmap
from PyQt5.QtCore import * # QTimer, QDateTime, Qt

import sqlite3

from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees

class statusButton:
    def __init__(self):
        # button left side and righ side
        self.bt_passAuto = 0
        self.bt_passHand = 0
        self.bt_cancelMission = 0
        self.bt_getInfo = 0
        self.bt_setting = 0
        self.bt_clearError = 0
        self.bt_exit = 0
        
        # button hand mode move page 1
        self.bt_changePage2 = 0
        self.bt_forwards = 0
        self.bt_backwards = 0
        self.bt_rotation_left = 0
        self.bt_rotation_right = 0
        self.bt_stop = 0
        self.bt_moveHand = 0
        
        # button hand mode move page 2
        self.bt_changePage1 = 0
        self.bt_chg_on = 0
        self.bt_chg_off = 0
        self.bt_spk_on = 0
        self.bt_spk_off = 0
        self.bt_speaker = 0
        self.bt_charger = 0
        self.bt_disableBrake = 0
        self.bt_setpose = 0

        # button auto mode
        self.bt_doNextPlan = 0
        self.bt_doNextPlanClicked = 1
        self.bt_doNextPlan2 = 0

        self.ck_remote = 0

        self.vs_speed = 50
        self.soundtype = 0
        self.ledtype = 0
        self.toyobit = 0

class statusColor:
    def __init__(self):
    # --
        self.lbc_safety_ahead = 0
        self.lbc_safety_behind = 0
        # --
        self.cb_status = 0
        self.lbc_battery = 0
        self.lbc_setpose = 0

        # --
        self.lbc_button_clearError = 0
        self.lbc_button_power = 0
        self.lbc_blsock = 0
        self.lbc_emg = 0

        self.lbc_limit_up = 0
        self.lbc_limit_down = 0
        self.lbc_detect_lifter = 0

        self.lbc_port_rtc = 0
        self.lbc_port_driverLeft = 0
        self.lbc_port_driverRight = 0
        self.lbc_port_lidar = 0
        self.lbc_port_rs485 = 0
        self.lbc_port_magLine = 0

        self.lbc_toyoReadBit0 = 0
        self.lbc_toyoReadBit1 = 0
        self.lbc_toyoReadBit2 = 0
        self.lbc_toyoReadBit3 = 0
        self.lbc_toyoReadBit4 = 0
        self.lbc_toyoReadBit5 = 0
        self.lbc_toyoReadBit6 = 0
        self.lbc_toyoReadBit7 = 0

        self.lbc_deletePose = 0
        self.lbc_addPose = 0

class valueLable:
    def __init__(self):
        self.modeRuning = 1

        self.lbv_name_agv = ''
        self.lbv_mac = ''
        self.lbv_ip = ''
        self.lbv_battery = ''
        self.lbv_date = ''

        self.lbv_coordinates_x = 0.
        self.lbv_coordinates_y = 0.
        self.lbv_coordinates_r = 0.
        self.lbv_pingServer = ''

        self.lbv_map_match_status = ''
        self.lbv_loc_status = ''

        self.lbv_errorPath = ''

        self.lbv_statusPath = 0

        self.lbv_goalFollow_id = ''
        self.lbv_route_message = ''
        self.lbv_jobRuning = ''
        self.lbv_jobDoing = 0

        self.lbv_mac = ''
        self.lbv_namePc = ''

        self.lbv_launhing = ''
        self.lbv_numberLaunch = ''
        self.percentLaunch = 0
        self.listError = ['A', 'B', 'C']
        self.listError_pre = []

        self.listIDSetpose = []
        self.isUpdateListIdSetpose = True

        self.listPath = []
        self.listPath_pre = []
        self.listFullPath = []
        self.typeTarget = 0

        # -- 
        self.list_indexTarget = []

        self.lbv_qualityWifi = 0
        # -
        self.list_logError = []
        self.list_logError_pre = []
        # -
        self.lbv_idPathFollow = 0
        self.lbv_pathVelocity = 0
        self.lbv_pathSafety = 0
        self.lbv_typeLineFollow = ''
        self.lbv_idtarget = 0
        self.lbv_xTarget = 0
        self.lbv_ytarget = 0
        self.lbv_rTarget = 0

        self.lbv_velLeft = ''
        self.lbv_velRight = ''
        self.lbv_notification_driver1 = ''
        self.lbv_notification_driver2 = ''

        self.lbv_cpu = ''
        self.lbv_cputemp=''
        self.lbv_ram=''
        self.lbv_uptime=''
        self.lbv_wifi=''

        self.lbv_coorAverage_x = ''
        self.lbv_coorAverage_y = ''
        self.lbv_coorAverage_r = ''
        self.lbv_coorAverage_times = ''
        self.lbv_deltaDistance = ''

        self.lbv_tryTarget_x = 0
        self.lbv_tryTarget_y = 0
        self.lbv_tryTarget_r = 0
        self.lbv_tryTarget_d = 0

        self.step_path = 0
        self.is_recvNNinfoRespond = 0
        self.status_show_nav = 0

class TypeLine(Enum):
    STRAIGHTLINE = 1
    CIRCLELINE = 2
    BEZIERLINE = 3

class TypeTarget(Enum):
	UNSTOP = 0
	STOP1 = 1
	STOP2 = 2
	STOP3 = 3

class infoPath:
    def __init__(self):
        self.typeLine = TypeLine
        self.startPoint = Point()
        self.endPoint = Point()
        self.midPoint = Point()
        self.isTarget = False

class CanvasWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.listPath = []
        self.xAGV = 0.
        self.yAGV = 0.
        self.rAGV = 0.

        self.isUpdatePath = False
        self.isUpdateAGV = False
        self.isUpdateFullPath = False

        self.OFFSET_WIDTH = 0      # OFFSET theo công thức    # 20
        self.OFFSET_HEIGHT = 0     # OFFSET Y theo công thức  # 15
        # self.OFFSET_X = -50        # OFFSET X hiển thị trên màn hình  for fakePlan sti
        # self.OFFSET_Y = 150        # OFFSET Y hiển thị trên màn hình
        # self.OFFSET_X = 230        # OFFSET X hiển thị trên màn hình 
        # self.OFFSET_Y = 50        # OFFSET Y hiển thị trên màn hình

        self.OFFSET_X = -250        # OFFSET X hiển thị trên màn hình 
        self.OFFSET_Y = 115        # OFFSET Y hiển thị trên màn hình

        self.dentaX = 0
        self.dentaY = 0
        self.max_X = 0.
        self.max_Y = 0.
        self.min_X = 0.
        self.min_Y = 0.
        self.min = 0.0
        self.WIDTH = 571 #m         # Độ rộng pixel của frame canvas
        self.HEIGHT = 311 #m        # Độ dài pixel của frame canvas
        self.SCREEN_WIDTH = 0.153 #m
        self.SCREEN_HEIGHT = 0.09 #m
        self.METER_TO_PIXEL = 3779.5275590551
        self.SCALE_COEF = 1         # Tỉ lệ hiển thị

        # path image
        self.path_greenPin = '/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/green_location_pin2.png'      # Điểm dừng 1
        self.path_bluePin = '/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/blue_location_pin2.png'       # Điểm dừng 2
        self.path_yellowPin = '/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/yellow_location_pin.png'     # Điểm dừng 3
        self.path_orangePin = '/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/orange_location_pin.png'     # Điểm dừng 3
        self.path_redPin = '/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/red_location_pin.png'     # Điểm dừng 3
        self.path_blackPin = '/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/black_location_pin.png'     # Điểm dừng 3
        self.path_purplePin = '/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/purple_location_pin.png'     # Điểm dừng 3

        self.pathIconTarget = '/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/toado.png'
        self.pathIconAGV = '/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/iconmuiten _resize.png'

        self.listFullPathcv = []
        self.typeTarget = 0
        self.list_indexTarget = []
        self.index_target1 = 0
        self.index_target2 = 3
        self.index_target3 = 11
        self.PIN_SMALL = 20
        self.PIN_BIG = 30
        self.PIN_OFFSET_X = 6    # offset for Pin small x
        self.PIN_OFFSET_Y = 26   # offset for Pin small y

        self.PINBIG_OFFSET_X = 10  # offset for Pin big x
        self.PINBIG_OFFSET_Y = 35  # offset for Pin big y

        self.AGV_WOFFSET = 5
        self.AGV_HOFFSET = 3

        self.ANGLE_TRANSLATE = 0.0

        self.iscreateAGV = 0

        self.CONTENT_POS_X = 100 
        self.CONTENT_POS_Y = 30

    def makeAGV(self, x_rb, y_rb, r_rb, r, h):
        x_cv = x_rb + int(r/2)
        y_cv = int(h/2) - y_rb
        # self.xAGV = self.OFFSET_WIDTH + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.OFFSET_WIDTH)/r) + self.OFFSET_X
        # self.yAGV = self.OFFSET_HEIGHT + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.OFFSET_HEIGHT)/h)
        # self.rAGV = 270 - r_rb

        X = self.OFFSET_WIDTH + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.OFFSET_WIDTH)/r)
        Y = self.OFFSET_HEIGHT + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.OFFSET_HEIGHT)/h)

        self.xAGV = int(X * cos(self.ANGLE_TRANSLATE) + Y * sin(self.ANGLE_TRANSLATE)) + self.OFFSET_X
        self.yAGV = int(-X * sin(self.ANGLE_TRANSLATE) + Y * cos(self.ANGLE_TRANSLATE)) + self.OFFSET_Y
        
        if r_rb <= 45 or (r_rb >= 135 and r_rb <= 225) or r_rb >= 315: 
            self.rAGV = r_rb + 90
        elif (r_rb > 45 and r_rb < 135) or (r_rb > 225 and r_rb < 315):
            self.rAGV = r_rb - 90

        # print("Tọa độ robot on Canvas là:", self.xAGV,"  " ,self.yAGV)

    def makePath(self, listPath, r, h):
        self.listPath = []
        n = len(listPath)
        x_cv = 0
        y_cv = 0
        for p in listPath:
            cvPath = infoPath()
            cvPath.typeLine = p.typeLine
            cvPath.isTarget = p.isTarget
            x_cv = p.startPoint.x + int(r/2)
            y_cv = int(h/2) - p.startPoint.y
            # cvPath.startPoint.x = self.OFFSET_WIDTH + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.OFFSET_WIDTH)/r) + self.OFFSET_X
            # cvPath.startPoint.y = self.OFFSET_HEIGHT + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.OFFSET_HEIGHT)/h)

            X = self.OFFSET_WIDTH + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.OFFSET_WIDTH)/r)
            Y = self.OFFSET_HEIGHT + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.OFFSET_HEIGHT)/h)	

            cvPath.startPoint.x = int(X * cos(self.ANGLE_TRANSLATE) + Y * sin(self.ANGLE_TRANSLATE)) + self.OFFSET_X
            cvPath.startPoint.y = int(-X * sin(self.ANGLE_TRANSLATE) + Y * cos(self.ANGLE_TRANSLATE))	+ self.OFFSET_Y	

            x_cv = p.endPoint.x + int(r/2)
            y_cv = int(h/2) - p.endPoint.y
            # cvPath.endPoint.x = self.OFFSET_WIDTH + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.OFFSET_WIDTH)/r) + self.OFFSET_X
            # cvPath.endPoint.y = self.OFFSET_HEIGHT + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.OFFSET_HEIGHT)/h)

            X = self.OFFSET_WIDTH + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.OFFSET_WIDTH)/r)
            Y = self.OFFSET_HEIGHT + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.OFFSET_HEIGHT)/h)	

            cvPath.endPoint.x = int(X * cos(self.ANGLE_TRANSLATE) + Y * sin(self.ANGLE_TRANSLATE)) + self.OFFSET_X
            cvPath.endPoint.y = int(-X * sin(self.ANGLE_TRANSLATE) + Y * cos(self.ANGLE_TRANSLATE)) + self.OFFSET_Y

            if p.typeLine == TypeLine.BEZIERLINE:
                x_cv = p.midPoint.x + int(r/2)
                y_cv = int(h/2) - p.midPoint.y

                X = self.OFFSET_WIDTH + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.OFFSET_WIDTH)/r)
                Y = self.OFFSET_HEIGHT + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.OFFSET_HEIGHT)/h)	

                cvPath.midPoint.x = int(X * cos(self.ANGLE_TRANSLATE) + Y * sin(self.ANGLE_TRANSLATE)) + self.OFFSET_X
                cvPath.midPoint.y = int(-X * sin(self.ANGLE_TRANSLATE) + Y * cos(self.ANGLE_TRANSLATE)) + self.OFFSET_Y

            self.listPath.append(cvPath)		
            
    def makeFullPath(self, listFullPath, r, h):
        self.listFullPathcv = []
        x_cv = 0
        y_cv = 0
        for p in listFullPath:
            cvPath = infoPath()
            cvPath.typeLine = p.typeLine
            x_cv = p.startPoint.x + int(r/2)
            y_cv = int(h/2) - p.startPoint.y

            # cvPath.startPoint.x = self.OFFSET_WIDTH + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.OFFSET_WIDTH)/r) + self.OFFSET_X
            # cvPath.startPoint.y = self.OFFSET_HEIGHT + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.OFFSET_HEIGHT)/h)
            X = self.OFFSET_WIDTH + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.OFFSET_WIDTH)/r)
            Y = self.OFFSET_HEIGHT + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.OFFSET_HEIGHT)/h)	

            cvPath.startPoint.x = int(X * cos(self.ANGLE_TRANSLATE) + Y * sin(self.ANGLE_TRANSLATE)) + self.OFFSET_X
            cvPath.startPoint.y = int(-X * sin(self.ANGLE_TRANSLATE) + Y * cos(self.ANGLE_TRANSLATE)) + self.OFFSET_Y

            x_cv = p.endPoint.x + int(r/2)
            y_cv = int(h/2) - p.endPoint.y
            # cvPath.endPoint.x = self.OFFSET_WIDTH + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.OFFSET_WIDTH)/r) + self.OFFSET_X 
            # cvPath.endPoint.y = self.OFFSET_HEIGHT + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.OFFSET_HEIGHT)/h)

            X = self.OFFSET_WIDTH + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.OFFSET_WIDTH)/r)
            Y = self.OFFSET_HEIGHT + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.OFFSET_HEIGHT)/h)	

            cvPath.endPoint.x = int(X * cos(self.ANGLE_TRANSLATE) + Y * sin(self.ANGLE_TRANSLATE)) + self.OFFSET_X
            cvPath.endPoint.y = int(-X * sin(self.ANGLE_TRANSLATE) + Y * cos(self.ANGLE_TRANSLATE)) + self.OFFSET_Y	

            if p.typeLine == TypeLine.BEZIERLINE.value:
                x_cv = p.midPoint.x + int(r/2)
                y_cv = int(h/2) - p.midPoint.y

                X = self.OFFSET_WIDTH + int(x_cv*self.SCALE_COEF*(self.WIDTH - 2*self.OFFSET_WIDTH)/r)
                Y = self.OFFSET_HEIGHT + int(y_cv*self.SCALE_COEF*(self.HEIGHT - 2*self.OFFSET_HEIGHT)/h)	

                cvPath.midPoint.x = int(X * cos(self.ANGLE_TRANSLATE) + Y * sin(self.ANGLE_TRANSLATE)) + self.OFFSET_X
                cvPath.midPoint.y = int(-X * sin(self.ANGLE_TRANSLATE) + Y * cos(self.ANGLE_TRANSLATE)) + self.OFFSET_Y
                
            self.listFullPathcv.append(cvPath)			

    def draw_text(self, painter, x, y, msg):
        pen = QPen()
        pen.setWidth(1)
        pen.setColor(QColor('black'))
        painter.setPen(pen)

        font = QFont()
        font.setFamily('Ubuntu')
        font.setBold(False)
        font.setItalic(True)
        font.setPointSize(10)
        painter.setFont(font)

        painter.drawText(x, y, msg)

    def draw_content(self, painter):
        self.draw_text(painter, self.CONTENT_POS_X, self.CONTENT_POS_Y, 'Điểm dừng 1')
        self.draw_text(painter, self.CONTENT_POS_X, self.CONTENT_POS_Y + 30, 'Điểm dừng 2')
        self.draw_text(painter, self.CONTENT_POS_X, self.CONTENT_POS_Y + 60, 'Điểm dừng 3')
        self.draw_text(painter, self.CONTENT_POS_X, self.CONTENT_POS_Y + 90, 'Điểm dừng 4')
        self.draw_text(painter, self.CONTENT_POS_X, self.CONTENT_POS_Y + 120, 'Điểm dừng 5')
        self.draw_text(painter, self.CONTENT_POS_X, self.CONTENT_POS_Y + 150, 'Điểm dừng 6')

        pixmap = QPixmap(self.path_yellowPin)
        pixmap1 = pixmap.scaled(self.PIN_SMALL, self.PIN_SMALL, QtCore.Qt.KeepAspectRatio)
        painter.drawPixmap(self.CONTENT_POS_X - 25, self.CONTENT_POS_Y - 20, pixmap1)

        pixmap = QPixmap(self.path_greenPin)
        pixmap1 = pixmap.scaled(self.PIN_SMALL, self.PIN_SMALL, QtCore.Qt.KeepAspectRatio)
        painter.drawPixmap(self.CONTENT_POS_X - 25, self.CONTENT_POS_Y + 10, pixmap1)

        pixmap = QPixmap(self.path_bluePin)
        pixmap1 = pixmap.scaled(self.PIN_SMALL, self.PIN_SMALL, QtCore.Qt.KeepAspectRatio)
        painter.drawPixmap(self.CONTENT_POS_X - 25, self.CONTENT_POS_Y + 40, pixmap1)

        pixmap = QPixmap(self.path_purplePin)
        pixmap1 = pixmap.scaled(self.PIN_SMALL, self.PIN_SMALL, QtCore.Qt.KeepAspectRatio)
        painter.drawPixmap(self.CONTENT_POS_X - 25, self.CONTENT_POS_Y + 70, pixmap1)

        pixmap = QPixmap(self.path_orangePin)
        pixmap1 = pixmap.scaled(self.PIN_SMALL, self.PIN_SMALL, QtCore.Qt.KeepAspectRatio)
        painter.drawPixmap(self.CONTENT_POS_X - 25, self.CONTENT_POS_Y + 100, pixmap1)

        pixmap = QPixmap(self.path_redPin)
        pixmap1 = pixmap.scaled(self.PIN_SMALL, self.PIN_SMALL, QtCore.Qt.KeepAspectRatio)
        painter.drawPixmap(self.CONTENT_POS_X - 25, self.CONTENT_POS_Y + 130, pixmap1)

    def update_Target(self, target):
        self.typeTarget = target

    def update_listTarget(self, ls_target):
        self.list_indexTarget = ls_target

    def paintEvent(self, event):
        # Create a painting object
        ThePainter = QPainter()
        # Do the painting
        ThePainter.begin(self) # everything between here and the end() call will be drawn into the Widget
        #self.draw_content(ThePainter)

        if self.isUpdateFullPath:
            self.isUpdateFullPath = False
            if len(self.listFullPathcv) > 0:			# Archie

                ThePen=QPen(Qt.gray) # set a color for a pen.  Pens draw lines and outlines
                for path in self.listFullPathcv:
                    if path.isTarget:
                        pixmap = QPixmap(self.pathIconTarget)
                        ThePainter.drawPixmap(path.endPoint.x - 15, path.endPoint.y - 30, pixmap)
                    else:
                        ThePen.setWidth(10)
                        ThePainter.setPen(ThePen) # set the pen into the painter
                        ThePainter.drawPoint(path.endPoint.x, path.endPoint.y)

                    ThePen.setWidth(6)
                    ThePainter.setPen(ThePen) # set the pen into the painter

                    if path.typeLine == TypeLine.STRAIGHTLINE.value:
                        ThePainter.drawLine(path.startPoint.x, path.startPoint.y, path.endPoint.x, path.endPoint.y)

                    elif path.typeLine == TypeLine.BEZIERLINE.value:
                        pathCubic = QPainterPath()
                        pathCubic.moveTo(path.startPoint.x, path.startPoint.y)
                        pathCubic.cubicTo(path.startPoint.x, path.startPoint.y, path.midPoint.x, path.midPoint.y, path.endPoint.x, path.endPoint.y)
                        ThePainter.drawPath(pathCubic)
                
                if len(self.list_indexTarget) > 0:
                    for index, target in enumerate(self.list_indexTarget):
                        if self.typeTarget == index:
                            size_pixmap = self.PIN_BIG
                            offset_pixmapx = self.PINBIG_OFFSET_X
                            offset_pixmapy = self.PINBIG_OFFSET_Y
                        else:
                            size_pixmap = self.PIN_SMALL
                            offset_pixmapx = self.PIN_OFFSET_X
                            offset_pixmapy = self.PIN_OFFSET_Y
                        
                        if index == 0:
                            pixmap = QPixmap(self.path_yellowPin)
                        elif index == 1:
                            pixmap = QPixmap(self.path_greenPin)
                        elif index == 2:
                            pixmap = QPixmap(self.path_bluePin)
                        elif index == 3:
                            pixmap = QPixmap(self.path_purplePin)
                        elif index == 4:
                            pixmap = QPixmap(self.path_orangePin)
                        elif index == 5:
                            pixmap = QPixmap(self.path_redPin)

                        pixmap1 = pixmap.scaled(size_pixmap, size_pixmap, QtCore.Qt.KeepAspectRatio)
                        ThePainter.drawPixmap(self.listFullPathcv[target].endPoint.x - offset_pixmapx, self.listFullPathcv[target].endPoint.y - offset_pixmapy, pixmap1)

        if self.isUpdatePath:
            self.isUpdatePath = False
            ThePen=QPen(Qt.green) # set a color for a pen.  Pens draw lines and outlines
            if len(self.listPath) > 0:
                for path in self.listPath:
                    ThePen.setWidth(2)
                    ThePainter.setPen(ThePen) # set the pen into the painter
                    if path.typeLine == TypeLine.STRAIGHTLINE:
                        ThePainter.drawLine(path.startPoint.x, path.startPoint.y, path.endPoint.x, path.endPoint.y)

                    elif path.typeLine == TypeLine.BEZIERLINE:
                        pathCubic = QPainterPath()
                        pathCubic.moveTo(path.startPoint.x, path.startPoint.y)
                        pathCubic.cubicTo(path.startPoint.x, path.startPoint.y, path.midPoint.x, path.midPoint.y, path.endPoint.x, path.endPoint.y)
                        ThePainter.drawPath(pathCubic)

        if self.isUpdateAGV:
            self.isUpdateAGV = False
            pixmap = QPixmap(self.pathIconAGV)
            transform = QTransform().rotate(self.rAGV)
            pixmap = pixmap.transformed(transform)
            ThePainter.drawPixmap(self.xAGV - 12, self.yAGV - 12, pixmap)

        ThePainter.end() # finishing drawing

class WelcomeScreen(QDialog):
    def __init__(self):
        super(WelcomeScreen, self).__init__()
        loadUi("/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/app_towing.ui", self)
        # --
        self.statusButton = statusButton()
        self.statusColor  = statusColor()
        self.valueLable   = valueLable()

        # -- 
        # Add a box layout into the frame
        SettingsLayout = QVBoxLayout()
        self.fr_pathPlan.setLayout(SettingsLayout)
        self.TheCanvas = CanvasWidget() # create the canvas widget that will do the painting
        SettingsLayout.addWidget(self.TheCanvas)  # add the canvas to the layout within the frame
        SettingsLayout.setContentsMargins(0, 0, 0, 0)
        # -
        self.cb_listToyo.setDisabled(True)
        self.bt_backwards.setDisabled(True)
        self.bt_backwards2.setDisabled(True)
        # -- frame left side ---------------------------------------
        self.bt_exit.pressed.connect(self.out)
        # -
        self.bt_cancelMission.pressed.connect(self.pressed_cancelMission)
        self.bt_cancelMission.released.connect(self.released_cancelMission)
        self.status_cancelMission = 0
        self.timeSave_cancelMisson = rospy.Time.now()
        # -
        self.bt_passHand.pressed.connect(self.pressed_passHand)
        self.bt_passHand.released.connect(self.released_passHand)
        # -
        self.bt_passAuto.pressed.connect(self.pressed_passAuto)
        self.bt_passAuto.released.connect(self.released_passAuto)
        # -
        # -- frame right side ---------------------------------------
        self.bt_getInfo.pressed.connect(self.pressed_getInfo)
        self.bt_getInfo.released.connect(self.released_getInfo)
        # -- 
        self.timeSave_getInfo = rospy.Time.now()
        self.status_getInfo = 0
        # -
        self.bt_clearError.pressed.connect(self.pressed_clearError)
        self.bt_clearError.released.connect(self.released_clearError)

        # -- Setting devices
        self.bt_setting.pressed.connect(self.pressed_setting)
        self.bt_setting.released.connect(self.released_setting)
        self.setting_status = 0
        self.isShow_setting = 0
        self.timeSave_setting = rospy.Time.now()
        # -
        self.bt_exit_settingFr.clicked.connect(self.clicked_exit_fr_setting)

        # -- page manual 1 ------------------------------------
        self.bt_forwards.clicked.connect(self.clicked_forwards)
        self.bt_backwards.clicked.connect(self.clicked_backwards)
        # -
        self.bt_rotation_left.clicked.connect(self.clicked_rotation_left)
        self.bt_rotation_right.clicked.connect(self.clicked_rotation_right)
        # -
        self.bt_stop.clicked.connect(self.clicked_stop)

        self.bt_forwards2.setStyleSheet('background-image: url(/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/arrow_small_up.png);background-position: center;background-repeat: no-repeat;')
        self.bt_backwards2.setStyleSheet('background-image: url(/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/arrow_small_down.png);background-position: center;background-repeat: no-repeat;')
        self.bt_rotation_left2.setStyleSheet('background-image: url(/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/arrow_small_left.png);background-position: center;background-repeat: no-repeat;')
        self.bt_rotation_right2.setStyleSheet('background-image: url(/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/arrow_small_right.png);background-position: center;background-repeat: no-repeat;')
        self.bt_stop2.setStyleSheet('background-image: url(/home/stivietnam/catkin_ws/src/app_ros/UI_AGVTowing/circle_small.png);background-position: center;background-repeat: no-repeat;')

        self.bt_forwards2.clicked.connect(self.clicked_forwards2)
        self.bt_backwards2.clicked.connect(self.clicked_backwards2)
        # -
        self.bt_rotation_left2.clicked.connect(self.clicked_rotation_left2)
        self.bt_rotation_right2.clicked.connect(self.clicked_rotation_right2)
        # -
        self.bt_stop2.clicked.connect(self.clicked_stop2)

        # -- page manual 2 -------------------------------------
        self.statusButton.bt_speaker = 1
        self.bt_speaker_on.clicked.connect(self.clicked_speaker_on)
        self.bt_speaker_off.clicked.connect(self.clicked_speaker_off)
        # --
        self.bt_charger_on.clicked.connect(self.clicked_charger_on)
        self.bt_charger_off.clicked.connect(self.clicked_charger_off)
        # -- 
        self.bt_disableBrake_on.clicked.connect(self.clicked_brakeOn)
        self.bt_disableBrake_off.clicked.connect(self.clicked_brakeOff)

        # self.bt_extendHandMode.pressed.connect(self.pressed_changePage1)
        # self.bt_extendHandMode.released.connect(self.released_changePage1)

        # self.bt_hideHandMode.pressed.connect(self.pressed_changePage2)
        # self.bt_hideHandMode.released.connect(self.released_changePage2)

        self.bt_extendHandMode.clicked.connect(self.clicked_extendHandMode)
        self.bt_hideHandMode.clicked.connect(self.clicked_hideHandMode)

        self.bt_setpose.pressed.connect(self.pressed_setpose)
        self.bt_setpose.released.connect(self.released_setpose)

        # -- frame auto
        self.bt_doNextPlan.pressed.connect(self.pressed_doNextPlan)
        self.bt_doNextPlan2.clicked.connect(self.clicked_doNextPlan2)
        self.bt_doNextPlan.released.connect(self.released_doNextPlan)

        # --
        self.bt_coorAverage.pressed.connect(self.pressed_bt_coorAverage)
        self.bt_coorAverage.released.connect(self.released_bt_coorAverage)

        self.bt_disPointA.clicked.connect(self.clicked_pointA)
        self.bt_disPointB.clicked.connect(self.clicked_pointB)

        # -
        self.bt_pw_cancel.clicked.connect(self.clicked_password_cancel)
        self.bt_pw_agree.clicked.connect(self.clicked_password_agree)
        self.password_data = ""
        self.password_right = "0111"
        self.bt_pw_0.clicked.connect(self.clicked_password_n0)
        self.bt_pw_1.clicked.connect(self.clicked_password_n1)
        self.bt_pw_2.clicked.connect(self.clicked_password_n2)
        self.bt_pw_3.clicked.connect(self.clicked_password_n3)
        self.bt_pw_4.clicked.connect(self.clicked_password_n4)
        self.bt_pw_5.clicked.connect(self.clicked_password_n5)
        self.bt_pw_6.clicked.connect(self.clicked_password_n6)
        self.bt_pw_7.clicked.connect(self.clicked_password_n7)
        self.bt_pw_8.clicked.connect(self.clicked_password_n8)
        self.bt_pw_9.clicked.connect(self.clicked_password_n9)
        self.bt_pw_clear.clicked.connect(self.clicked_password_clear)
        self.bt_pw_delete.clicked.connect(self.clicked_password_delete)

        self.ck_remote.stateChanged.connect(self.stateChanged_ck_remote)

        # -
        self.bt_exit_fr_infoDevice.clicked.connect(self.clicked_exit_fr_info)
        self.bt_exit_fr_infoModeAuto.clicked.connect(self.clicked_exit_fr_info)

        # -- Set speed manual
        self.bt_upSpeed.pressed.connect(self.pressed_upSpeed)
        self.bt_reduceSpeed.pressed.connect(self.pressed_reduceSpeed)

        # -- combo Box of speaker
        self.cb_listSpeak.addItem(" Tắt loa")
        self.cb_listSpeak.addItem(" Âm 1 - Âm chờ")
        self.cb_listSpeak.addItem(" Âm 2 - Bình Thường")
        self.cb_listSpeak.addItem(" Âm 3 - Rẽ Trái")
        self.cb_listSpeak.addItem(" Âm 4 - Rẽ Phải")
        self.cb_listSpeak.addItem(" Âm 5 - Lỗi Vật Cản")
        self.cb_listSpeak.addItem(" Âm 6 - Cảnh Báo")
        self.cb_listSpeak.addItem(" Âm 7 - Dừng Khẩn")        
        self.cb_listSpeak.addItem(" Âm 8 - Lỗi hệ thống")

        self.cb_listLed.addItem(" Tắt led")
        self.cb_listLed.addItem(" Led 1 - Bình Thường")
        self.cb_listLed.addItem(" Led 2 - Rẽ Trái")
        self.cb_listLed.addItem(" Led 3 - Rẽ Phải")
        self.cb_listLed.addItem(" Led 4 - Lỗi Vật Cản")
        self.cb_listLed.addItem(" Led 5 - Cảnh Báo")
        self.cb_listLed.addItem(" Led 6 - Dừng Khẩn")        
        #self.cb_listLed.addItem(" Led 7 - Lỗi hệ thống")

        self.cb_listToyo.addItem(" Tắt Toyo")
        self.cb_listToyo.addItem(" Bit 1")
        self.cb_listToyo.addItem(" Bit 2")
        self.cb_listToyo.addItem(" Bit 3")
        self.cb_listToyo.addItem(" Bit 4")
        self.cb_listToyo.addItem(" Bit 5")
        self.cb_listToyo.addItem(" Bit 6")        
        self.cb_listToyo.addItem(" Bit 7")
        self.cb_listToyo.addItem(" Bit 8")

        # -- edit set pose
        # -
        self.bt_tryTarget_x.pressed.connect(self.pressed_tryTarget_x)
        # -
        self.bt_tryTarget_y.pressed.connect(self.pressed_tryTarget_y)
        # -
        self.bt_tryTarget_r.pressed.connect(self.pressed_tryTarget_r)
        # -
        self.bt_tryTarget_d.pressed.connect(self.pressed_tryTarget_d)

        # -
        self.bt_tryTarget_up.pressed.connect(self.pressed_tryTarget_up)
        self.bt_tryTarget_up.released.connect(self.released_tryTarget_up)
        # # -
        self.bt_tryTarget_down.pressed.connect(self.pressed_tryTarget_down)
        self.bt_tryTarget_down.released.connect(self.released_tryTarget_down)

        self.bt_addPose.clicked.connect(self.clicked_addPose)
        self.bt_deletePose.clicked.connect(self.clicked_deletePose)

        # -- -- -- Timer updata data
        # -- Fast
        timer_fast = QTimer(self)
        timer_fast.timeout.connect(self.process_fast)
        timer_fast.start(50)
        # -- normal
        timer_normal = QTimer(self)
        timer_normal.timeout.connect(self.process_normal)
        timer_normal.start(996)
        # -- Slow
        timer_slow = QTimer(self)
        timer_slow.timeout.connect(self.process_slow)
        timer_slow.start(3000)
        # --
        self.modeRuning = 0
        self.modeRun_launch = 0
        self.modeRun_byhand = 1
        self.modeRun_auto = 2
        self.modeRun_cancelMission = 5

        self.modeRuning = self.modeRun_byhand               # Archie
        self.saveModeRuning = self.modeRun_byhand
        # --
        self.password_data = ""
        # - 
        self.handMode1 = 1
        self.handMode2 = 2
        self.handMode = self.handMode1

        # -
        self.robotPoseNow = Pose()
        self.pointA = Point()
        self.pointB = Point()
        # -
        self.bt_coorAverage_status = 0
        # -
        self.countTime_coorAverage = 0
        self.total_x = 0.0
        self.total_y = 0.0
        self.total_angle = 0.0

        self.flag_addPose = 0
        self.flag_deletePose = 0

        self.runOnce = 1
        self.runOnce1 = 1
        self.ratio_w = 150
        self.ratio_h = 50

        self.status_show_nav = 0
        self.status_show_setting = 0

    def out(self):
        QApplication.quit()
        print('out')

    def stateChanged_ck_remote(self):
        self.clicked_stop()

    def pressed_cancelMission(self):
        self.bt_cancelMission.setStyleSheet("background-color: blue;")
        self.statusButton.bt_cancelMission = 1

    def released_cancelMission(self):
        self.bt_cancelMission.setStyleSheet("background-color: white;")
        self.statusButton.bt_cancelMission = 0
        self.isShow_setting = 0

    def pressed_passHand(self):
        self.statusButton.bt_passHand = 1
        self.bt_passHand.setStyleSheet("background-color: blue;")

    def released_passHand(self):
        self.statusButton.bt_passHand = 0
        self.bt_passHand.setStyleSheet("background-color: white;")
        self.isShow_setting = 0

    def pressed_passAuto(self):
        self.statusButton.bt_passAuto = 1
        self.clicked_stop()
        # --
        self.statusButton.bt_disableBrake = 0
        self.bt_disableBrake_off.setStyleSheet("background-color: blue;")
        self.bt_disableBrake_on.setStyleSheet("background-color: white;")

    def released_passAuto(self):
        self.statusButton.bt_passAuto = 0
        self.isShow_setting = 0

    #  --
    def pressed_getInfo(self):
        self.statusButton.bt_getInfo = 1
        self.clicked_stop()
        # --
        self.bt_getInfo.setStyleSheet("background-color: blue;")

    def released_getInfo(self):
        self.statusButton.bt_getInfo = 0
        self.bt_getInfo.setStyleSheet("background-color: white;")

    def clicked_exit_fr_info(self):
        self.status_getInfo = 0
        self.statusButton.bt_getInfo = 0

    def pressed_clearError(self):
        self.statusButton.bt_clearError = 1
        self.bt_clearError.setStyleSheet("background-color: blue;")
        self.clicked_stop()

    def released_clearError(self):
        self.statusButton.bt_clearError = 0
        self.bt_clearError.setStyleSheet("background-color: white;")
        self.clicked_stop()

    def clicked_speaker_on(self):
        # self.statusButton.bt_spk_on = 1
        # self.statusButton.bt_spk_off = 0
        self.statusButton.bt_speaker = 1
        self.bt_speaker_on.setStyleSheet("background-color: blue;")
        self.bt_speaker_off.setStyleSheet("background-color: white;")
        self.clicked_stop()

    def clicked_speaker_off(self):
        # self.statusButton.bt_spk_on = 0
        # self.statusButton.bt_spk_off = 1
        self.statusButton.bt_speaker = 0
        self.bt_speaker_off.setStyleSheet("background-color: blue;")
        self.bt_speaker_on.setStyleSheet("background-color: white;")
        self.clicked_stop()

    def clicked_charger_on(self):
        # self.statusButton.bt_chg_on = 1
        # self.statusButton.bt_chg_off = 0
        self.statusButton.bt_charger = 1
        self.bt_charger_on.setStyleSheet("background-color: blue;")
        self.bt_charger_off.setStyleSheet("background-color: white;")
        self.clicked_stop()

    def clicked_charger_off(self):
        # self.statusButton.bt_chg_on = 0
        # self.statusButton.bt_chg_off = 1
        self.statusButton.bt_charger = 0
        self.bt_charger_off.setStyleSheet("background-color: blue;")
        self.bt_charger_on.setStyleSheet("background-color: white;")
        self.clicked_stop()

    def clicked_brakeOn(self):
        self.statusButton.bt_disableBrake = 1
        self.bt_disableBrake_on.setStyleSheet("background-color: blue;")
        self.bt_disableBrake_off.setStyleSheet("background-color: white;")
        self.clicked_stop()

    def clicked_brakeOff(self):
        self.statusButton.bt_disableBrake = 0
        self.bt_disableBrake_off.setStyleSheet("background-color: blue;")
        self.bt_disableBrake_on.setStyleSheet("background-color: white;")
        self.clicked_stop()

    # -
    def pressed_setpose(self):
        self.statusButton.bt_setpose = 1
        # self.statusColor.lbc_setpose = 2
        self.clicked_stop()
        
    def released_setpose(self):
        self.statusButton.bt_setpose = 0
        # self.statusColor.lbc_setpose = 0
        self.clicked_stop()

    def clicked_forwards(self):
        # self.statusButton.bt_forwards = 1
        # self.statusButton.bt_backwards = 0
        # self.statusButton.bt_rotation_left = 0
        # self.statusButton.bt_rotation_right = 0
        # self.statusButton.bt_stop = 0
        self.statusButton.bt_moveHand = 1
        self.bt_forwards.setStyleSheet("background-color: blue;")
        self.bt_backwards.setStyleSheet("background-color: white;")
        self.bt_rotation_left.setStyleSheet("background-color: white;")
        self.bt_rotation_right.setStyleSheet("background-color: white;")
        self.bt_stop.setStyleSheet("background-color: white;")
    # - 
    def clicked_backwards(self):
        # self.statusButton.bt_backwards = 1
        # self.statusButton.bt_forwards = 0
        # self.statusButton.bt_rotation_left = 0
        # self.statusButton.bt_rotation_right = 0
        # self.statusButton.bt_stop = 0
        self.statusButton.bt_moveHand = 2
        self.bt_backwards.setStyleSheet("background-color: blue;")
        self.bt_forwards.setStyleSheet("background-color: white;")
        self.bt_rotation_left.setStyleSheet("background-color: white;")
        self.bt_rotation_right.setStyleSheet("background-color: white;")
        self.bt_stop.setStyleSheet("background-color: white;")
    # - 
    def clicked_rotation_left(self):
        # self.statusButton.bt_rotation_left = 1
        # self.statusButton.bt_forwards = 0
        # self.statusButton.bt_backwards = 0
        # self.statusButton.bt_rotation_right = 0
        # self.statusButton.bt_stop = 0
        self.statusButton.bt_moveHand = 3
        self.bt_rotation_left.setStyleSheet("background-color: blue;")
        self.bt_forwards.setStyleSheet("background-color: white;")
        self.bt_backwards.setStyleSheet("background-color: white;")
        self.bt_rotation_right.setStyleSheet("background-color: white;")
        self.bt_stop.setStyleSheet("background-color: white;")
    # - 
    def clicked_rotation_right(self):
        # self.statusButton.bt_rotation_right = 1
        # self.statusButton.bt_forwards = 0
        # self.statusButton.bt_rotation_left = 0
        # self.statusButton.bt_backwards = 0
        # self.statusButton.bt_stop = 0
        self.statusButton.bt_moveHand = 4
        self.bt_rotation_right.setStyleSheet("background-color: blue;")
        self.bt_forwards.setStyleSheet("background-color: white;")
        self.bt_backwards.setStyleSheet("background-color: white;")
        self.bt_rotation_left.setStyleSheet("background-color: white;")
        self.bt_stop.setStyleSheet("background-color: white;")
    # - 
    def clicked_stop(self):
        # self.statusButton.bt_stop = 1
        # self.statusButton.bt_forwards = 0
        # self.statusButton.bt_rotation_left = 0
        # self.statusButton.bt_rotation_right = 0
        # self.statusButton.bt_backwards = 0

        self.statusButton.bt_moveHand = 0

        self.bt_forwards.setStyleSheet("background-color: white;")
        self.bt_backwards.setStyleSheet("background-color: white;")
        self.bt_rotation_right.setStyleSheet("background-color: white;")
        self.bt_rotation_left.setStyleSheet("background-color: white;")
        self.bt_stop.setStyleSheet("background-color: blue;")

    def clicked_forwards2(self):
        self.statusButton.bt_moveHand = 1
    # - 
    def clicked_backwards2(self):
        self.statusButton.bt_moveHand = 2
    # - 
    def clicked_rotation_left2(self):
        self.statusButton.bt_moveHand = 3
    # - 
    def clicked_rotation_right2(self):
        self.statusButton.bt_moveHand = 4
    # - 
    def clicked_stop2(self):
        self.statusButton.bt_moveHand = 0

    def pressed_bt_coorAverage(self):
        self.bt_coorAverage.setStyleSheet("background-color: blue;")
        self.bt_coorAverage_status = 1

    def released_bt_coorAverage(self):
        self.bt_coorAverage.setStyleSheet("background-color: white;")
        self.bt_coorAverage_status = 0

    def clicked_pointA(self):
        self.pointA.x = self.robotPoseNow.position.x
        self.pointA.y = self.robotPoseNow.position.y
        self.lbv_deltaDistance.setText("---")

    def clicked_pointB(self):
        self.pointB.x = self.robotPoseNow.position.x
        self.pointB.y = self.robotPoseNow.position.y
        delta_distance = self.calculate_distance(self.pointA, self.pointB)
        self.lbv_deltaDistance.setText( str(round(delta_distance, 3)) )

    def clicked_extendHandMode(self):
        self.handMode = self.handMode2

    def clicked_hideHandMode(self):
        self.handMode = self.handMode1

    def pressed_doNextPlan(self):
        self.statusButton.bt_doNextPlan = 1
        # self.bt_doNextPlan.setStyleSheet("background-color: blue;")

    def released_doNextPlan(self):
        self.statusButton.bt_doNextPlan = 0
        self.statusButton.bt_doNextPlanClicked = 1 - self.statusButton.bt_doNextPlanClicked
        # self.bt_doNextPlan.setStyleSheet("background-color: rgb(80,249,255);")

    def clicked_doNextPlan2(self):
        self.statusButton.bt_doNextPlan2 = 1

    def pressed_upSpeed(self):
        self.statusButton.vs_speed += 10
        if self.statusButton.vs_speed >= 100:
            self.statusButton.vs_speed = 100

    def pressed_reduceSpeed(self):
        self.statusButton.vs_speed -= 10
        if self.statusButton.vs_speed < 5:
            self.statusButton.vs_speed = 5

    # --  --
    def clicked_password_agree(self):
        self.password_data = ""
        self.status_cancelMission = 0
        self.status_show_setting = 1

    def clicked_password_cancel(self):
        self.password_data = ""
        self.status_cancelMission = 0
        self.status_show_setting = 0
        self.isShow_setting = 0
        
    def clicked_exit_fr_setting(self):
        self.isShow_setting = 0
        self.status_show_setting = 0
        self.password_data = ""

    def pressed_setting(self):
        self.bt_setting.setStyleSheet("background-color: blue;")	
        self.setting_status = 1

    def released_setting(self):
        self.bt_setting.setStyleSheet("background-color: white;")	
        self.setting_status = 0
    # --
    def pressed_tryTarget_x(self):
        self.bt_tryTarget_x.setStyleSheet("background-color: blue;")
        self.bt_tryTarget_y.setStyleSheet("background-color: white;")
        self.bt_tryTarget_r.setStyleSheet("background-color: white;")
        self.bt_tryTarget_d.setStyleSheet("background-color: white;")
        self.show_combox_unitMeter()
        self.changeNow = 1

    def pressed_tryTarget_y(self):
        self.bt_tryTarget_x.setStyleSheet("background-color: white;")
        self.bt_tryTarget_y.setStyleSheet("background-color: blue;")
        self.bt_tryTarget_r.setStyleSheet("background-color: white;")
        self.bt_tryTarget_d.setStyleSheet("background-color: white;")
        self.show_combox_unitMeter()
        self.changeNow = 2

    def pressed_tryTarget_r(self):
        self.bt_tryTarget_x.setStyleSheet("background-color: white;")
        self.bt_tryTarget_y.setStyleSheet("background-color: white;")
        self.bt_tryTarget_r.setStyleSheet("background-color: blue;")
        self.bt_tryTarget_d.setStyleSheet("background-color: white;")
        self.show_combox_unitDegree()
        self.changeNow = 3

    def pressed_tryTarget_d(self):
        self.bt_tryTarget_x.setStyleSheet("background-color: white;")
        self.bt_tryTarget_y.setStyleSheet("background-color: white;")
        self.bt_tryTarget_r.setStyleSheet("background-color: white;")
        self.bt_tryTarget_d.setStyleSheet("background-color: blue;")
        self.show_combox_unitMeter2()
        self.changeNow = 4

    def show_combox_unitMeter(self):
        self.cb_unit.clear()
        self.lbv_unit.setText("m")
        self.cb_unit.addItem("0.001")
        self.cb_unit.addItem("0.002")
        self.cb_unit.addItem("0.005")
        self.cb_unit.addItem("0.01")
        self.cb_unit.addItem("0.1")
        self.cb_unit.addItem("1")
        self.cb_unit.addItem("2")
        self.cb_unit.addItem("5")

    def show_combox_unitMeter2(self):
        self.cb_unit.clear()
        self.lbv_unit.setText("m")
        self.cb_unit.addItem("1")
        self.cb_unit.addItem("2")
        self.cb_unit.addItem("5")        

    def show_combox_unitDegree(self):
        self.cb_unit.clear()
        self.lbv_unit.setText("Độ")
        self.cb_unit.addItem("0.01")
        self.cb_unit.addItem("0.1")
        self.cb_unit.addItem("0.2")
        self.cb_unit.addItem("0.5")
        self.cb_unit.addItem("1")
        self.cb_unit.addItem("2")
        self.cb_unit.addItem("5")

    # --
    def pressed_tryTarget_up(self):
        self.bt_tryTarget_up.setStyleSheet("background-color: blue;")
        if self.cb_unit.currentText() != '':
            val_change = float(self.cb_unit.currentText())
            # - X
            if self.changeNow == 1:
                self.valueLable.lbv_tryTarget_x += val_change
            # - Y
            if self.changeNow == 2:
                self.valueLable.lbv_tryTarget_y += val_change
            # - R
            if self.changeNow == 3:
                self.valueLable.lbv_tryTarget_r += val_change
            # - D
            if self.changeNow == 4:
                self.valueLable.lbv_tryTarget_d += val_change
                self.valueLable.lbv_tryTarget_d = int(self.valueLable.lbv_tryTarget_d)
            
    def released_tryTarget_up(self):
        self.bt_tryTarget_up.setStyleSheet("background-color: white;")
        self.lbv_tryTarget_x.setText(str(round(self.valueLable.lbv_tryTarget_x, 3)))
        self.lbv_tryTarget_y.setText(str(round(self.valueLable.lbv_tryTarget_y, 3)))
        self.lbv_tryTarget_r.setText(str(round(self.valueLable.lbv_tryTarget_r, 3)))
        self.lbv_tryTarget_d.setText(str(self.valueLable.lbv_tryTarget_d))

    # --
    def pressed_tryTarget_down(self):
        self.bt_tryTarget_down.setStyleSheet("background-color: blue;")
        val_change = float(self.cb_unit.currentText())
        # - X
        if self.changeNow == 1:
            self.valueLable.lbv_tryTarget_x -= val_change
        # - Y
        if self.changeNow == 2:
            self.valueLable.lbv_tryTarget_y -= val_change
        # - R
        if self.changeNow == 3:
            self.valueLable.lbv_tryTarget_r -= val_change
        # - D
        if self.changeNow == 4:
            self.valueLable.lbv_tryTarget_d -= val_change
            if self.valueLable.lbv_tryTarget_d <= 0:
                self.valueLable.lbv_tryTarget_d = 0 

            self.valueLable.lbv_tryTarget_d = int(self.valueLable.lbv_tryTarget_d)

    def released_tryTarget_down(self):
        self.bt_tryTarget_down.setStyleSheet("background-color: white;")
        self.lbv_tryTarget_x.setText(str(round(self.valueLable.lbv_tryTarget_x, 3)))
        self.lbv_tryTarget_y.setText(str(round(self.valueLable.lbv_tryTarget_y, 3)))
        self.lbv_tryTarget_r.setText(str(round(self.valueLable.lbv_tryTarget_r, 3)))
        self.lbv_tryTarget_d.setText(str(self.valueLable.lbv_tryTarget_d))

    def clicked_addPose(self):
        self.flag_addPose = 1

    def clicked_deletePose(self):
        self.flag_deletePose = 1

    # - 
    def clicked_password_n0(self):
        if (len(self.password_data) < 4):
            self.password_data += "0"

    def clicked_password_n1(self):
        if (len(self.password_data) < 4):
            self.password_data += "1"

    def clicked_password_n2(self):
        if (len(self.password_data) < 4):
            self.password_data += "2"

    def clicked_password_n3(self):
        if (len(self.password_data) < 4):
            self.password_data += "3"

    def clicked_password_n4(self):
        if (len(self.password_data) < 4):
            self.password_data += "4"

    def clicked_password_n5(self):
        if (len(self.password_data) < 4):
            self.password_data += "5"

    def clicked_password_n6(self):
        if (len(self.password_data) < 4):
            self.password_data += "6"

    def clicked_password_n7(self):
        if (len(self.password_data) < 4):
            self.password_data += "7"

    def clicked_password_n8(self):
        if (len(self.password_data) < 4):
            self.password_data += "8"

    def clicked_password_n9(self):
        if (len(self.password_data) < 4):
            self.password_data += "9"

    def clicked_password_clear(self):
        self.password_data = ""

    def clicked_password_delete(self):
        lenght = len(self.password_data)
        if (lenght > 0):
            self.password_data = self.password_data[:-1]

    # Show label
    def set_dateTime(self):
        time_now = datetime.now()
        # dd/mm/YY H:M:S
        dt_string = time_now.strftime("%d/%m/%Y\n%H:%M:%S")
        self.lbv_date.setText(dt_string)

    def set_labelValue(self): # App_lbv()
        self.pb_qualityWifi.setValue(self.valueLable.lbv_qualityWifi)
        self.pb_speed.setValue(self.statusButton.vs_speed)
        # --

        self.lbv_coordinates_x.setText(str(self.valueLable.lbv_coordinates_x))
        self.lbv_coordinates_y.setText(str(self.valueLable.lbv_coordinates_y))
        self.lbv_coordinates_r.setText(str(self.valueLable.lbv_coordinates_r))

        self.lbv_pingServer.setText(self.valueLable.lbv_pingServer)
        self.lbv_pingServer2.setText(self.valueLable.lbv_pingServer)
        self.lbv_jobRuning.setText(self.valueLable.lbv_jobRuning)
        self.lbv_route_message.setText(self.valueLable.lbv_route_message)

        self.lbv_status_lidarLoc.setText(self.valueLable.lbv_loc_status)
        self.lbv_map_match.setText(self.valueLable.lbv_map_match_status)

        self.lbv_errorPath.setText(self.valueLable.lbv_errorPath)

        if self.valueLable.lbv_statusPath == 1:
            self.lbv_errorPath.setStyleSheet("background-color: red;")
        else:
            self.lbv_errorPath.setStyleSheet("background-color: none;")

        self.lbv_velLeft.setText(self.valueLable.lbv_velLeft)
        self.lbv_velRight.setText(self.valueLable.lbv_velRight)
        self.lbv_notification_driver1.setText(self.valueLable.lbv_notification_driver1)
        self.lbv_notification_driver2.setText(self.valueLable.lbv_notification_driver2)

        self.lbv_cpu.setText(self.valueLable.lbv_cpu)
        self.lbv_cpu2.setText(self.valueLable.lbv_cpu)
        self.lbv_tempCpu.setText(self.valueLable.lbv_cputemp)
        self.lbv_ram.setText(self.valueLable.lbv_ram)
        self.lbv_runtime.setText(self.valueLable.lbv_uptime)
        self.lbv_wifi.setText(self.valueLable.lbv_wifi)

        # -- 
        self.lbv_idPathFollow.setText(str(self.valueLable.lbv_idPathFollow))
        self.lbv_pathVelocity.setText(str(self.valueLable.lbv_pathVelocity))
        self.lbv_pathSafety.setText(str(self.valueLable.lbv_pathSafety))
        self.lbv_typeLineFollow.setText(self.valueLable.lbv_typeLineFollow)

        # --
        self.lbv_idtarget.setText(str(self.valueLable.lbv_idtarget))
        self.lbv_xTarget.setText(str(self.valueLable.lbv_xTarget))
        self.lbv_ytarget.setText(str(self.valueLable.lbv_ytarget))
        self.lbv_rTarget.setText(str(self.valueLable.lbv_rTarget))

    def set_labelColor(self):
        # ---- Safety
        # -
        if (self.statusColor.lbc_safety_ahead == 0):
            self.lbc_safety1.hide()
            self.lbc_safety2.hide()
            self.lbc_safety3.hide()
            # self.lbc_safety1.setStyleSheet("background-color: green; color: white")
            # self.lbc_safety_ahead.setStyleSheet("background-color: green; color: white")

        elif (self.statusColor.lbc_safety_ahead == 1):
            self.lbc_safety1.hide()
            self.lbc_safety2.hide()
            self.lbc_safety3.show()
            # self.lbc_safety1.setStyleSheet("background-color: red; color: white")
            # self.lbc_safety_ahead.setStyleSheet("background-color: red; color: white")
        elif (self.statusColor.lbc_safety_ahead == 2):
            self.lbc_safety1.hide()
            self.lbc_safety2.show()
            self.lbc_safety3.hide()
            # self.lbc_safety1.setStyleSheet("background-color: orange; color: white")
            # self.lbc_safety_ahead.setStyleSheet("background-color: red; color: white")
        else:
            self.lbc_safety1.show()
            self.lbc_safety2.hide()
            self.lbc_safety3.hide()
            # self.lbc_safety_ahead.setStyleSheet("background-color: yellow;")
        # -
        # if (self.statusColor.lbc_safety_behind == 0):
        #     self.lbc_safety2.setStyleSheet("background-color: green; color: white")
        #     # self.lbc_safety_behind.setStyleSheet("background-color: green; color: white")

        # elif (self.statusColor.lbc_safety_behind == 1):
        #     self.lbc_safety2.setStyleSheet("background-color: red; color: white")
        #     # self.lbc_safety_behind.setStyleSheet("background-color: red; color: white")
        # elif (self.statusColor.lbc_safety_behind == 2):
        #     self.lbc_safety2.setStyleSheet("background-color: orange; color: white")
        #     # self.lbc_safety_behind.setStyleSheet("background-color: red; color: white")
        # else:
        #     self.lbc_safety2.setStyleSheet("background-color: yellow;")
            # self.lbc_safety_behind.setStyleSheet("background-color: yellow;")

        # ---- Battery
        if (self.statusColor.lbc_battery == 0):
            self.lbv_battery.setStyleSheet("background-color: white; color: black;")
            self.lb_v.setStyleSheet("color: black;")
        elif (self.statusColor.lbc_battery == 1):
            self.lbv_battery.setStyleSheet("background-color: green; color: white;")
            self.lb_v.setStyleSheet("color: white;")
        elif (self.statusColor.lbc_battery == 2):
            self.lbv_battery.setStyleSheet("background-color: orange; color: black;")
            self.lb_v.setStyleSheet("color: black;")
        elif (self.statusColor.lbc_battery == 3):
            self.lbv_battery.setStyleSheet("background-color: red; color: white;")
            self.lb_v.setStyleSheet("color: white;")
        elif (self.statusColor.lbc_battery == 4):
            self.lbv_battery.setStyleSheet("background-color: yellow; color: black;")
            self.lb_v.setStyleSheet("color: black;")
        else: # -- charging
            self.lbv_battery.setStyleSheet("background-color: white; color: black;")
            self.lb_v.setStyleSheet("color: black;")

        # ---- Thanh trang thai AGV
        if (self.statusColor.cb_status == 0):
            self.cb_status.setStyleSheet("background-color: green; color: white;")
        elif (self.statusColor.cb_status == 1):
            self.cb_status.setStyleSheet("background-color: orange; color: black;")	
        elif (self.statusColor.cb_status == 2):
            self.cb_status.setStyleSheet("background-color: red; color: white;")
        else:
            self.cb_status.setStyleSheet("background-color: white;")

        # ---- Trang thai che do dang hoat dong
        if (self.valueLable.modeRuning == self.modeRun_byhand):
            self.bt_passHand.setStyleSheet("background-color: blue;")	
            self.bt_passAuto.setStyleSheet("background-color: white;")	

        elif (self.valueLable.modeRuning == self.modeRun_auto):
            self.bt_passHand.setStyleSheet("background-color: white;")	
            self.bt_passAuto.setStyleSheet("background-color: blue;")
        else:	
            self.bt_passHand.setStyleSheet("background-color: white;")	
            self.bt_passAuto.setStyleSheet("background-color: white;")

        # -- Button Clear error
        if (self.statusColor.lbc_button_clearError == 1):
            self.lbc_button_clearError.setStyleSheet("background-color: blue;")
        elif (self.statusColor.lbc_button_clearError == 0):
            self.lbc_button_clearError.setStyleSheet("background-color: white;")

        # -- Button Power
        if (self.statusColor.lbc_button_power == 1):
            self.lbc_button_power.setStyleSheet("background-color: blue;")
        elif (self.statusColor.lbc_button_power == 0):
            self.lbc_button_power.setStyleSheet("background-color: white;")

        # -- Blsock
        if (self.statusColor.lbc_blsock == 1):
            self.lbc_blsock.setStyleSheet("background-color: blue;")
        elif (self.statusColor.lbc_blsock == 0):
            self.lbc_blsock.setStyleSheet("background-color: white;")

        # -- EMG
        if (self.statusColor.lbc_emg == 1):
            self.lbc_emg.setStyleSheet("background-color: blue;")
        elif (self.statusColor.lbc_emg == 0):
            self.lbc_emg.setStyleSheet("background-color: white;")	

        # -- Port: RTC Board
        if (self.statusColor.lbc_port_rtc == 1):
            self.lbc_port_rtc.setStyleSheet("background-color: blue; color: white")
        elif (self.statusColor.lbc_port_rtc == 0):
            self.lbc_port_rtc.setStyleSheet("background-color: red; color: white;")

        # -- Port: RS485
        if (self.statusColor.lbc_port_rs485 == 1):
            self.lbc_port_rs485.setStyleSheet("background-color: blue; color: white")
        elif (self.statusColor.lbc_port_rs485 == 0):
            self.lbc_port_rs485.setStyleSheet("background-color: red; color: white;")

        # -- Port: NanoScan3
        if (self.statusColor.lbc_port_lidar == 1):
            self.lbc_port_lidar.setStyleSheet("background-color: blue; color: white")
        elif (self.statusColor.lbc_port_lidar == 0):
            self.lbc_port_lidar.setStyleSheet("background-color: red; color: white;")

        # -- Port: Magnetic Line
        if (self.statusColor.lbc_port_magLine == 1):
            self.lbc_port_magLine.setStyleSheet("background-color: blue; color: white")
        elif (self.statusColor.lbc_port_magLine == 0):
            self.lbc_port_magLine.setStyleSheet("background-color: red; color: white;")

        # -- Sensor Up
        # if (self.statusColor.lbc_limit_up == 1):
        #     self.lbc_limit_up.setStyleSheet("background-color: blue; color: white;")
        # elif (self.statusColor.lbc_limit_up == 0):
        #     self.lbc_limit_up.setStyleSheet("background-color: white; color: black;")

        # -- Sensor Down
        # if (self.statusColor.lbc_limit_down == 1):
        #     self.lbc_limit_down.setStyleSheet("background-color: blue; color: white;")
        # elif (self.statusColor.lbc_limit_down == 0):
        #     self.lbc_limit_down.setStyleSheet("background-color: white; color: black;")

        # -- Sensor detect lift
        # if (self.statusColor.lbc_detect_lifter == 1):
        #     self.lbc_detect_lifter.setStyleSheet("background-color: blue; color: white;")
        # elif (self.statusColor.lbc_detect_lifter == 0):
        #     self.lbc_detect_lifter.setStyleSheet("background-color: white; color: black;")

        # --
        self.statusButton.ck_remote = self.ck_remote.isChecked()
        # --
        if self.statusColor.lbc_setpose == 0:
            self.bt_setpose.setStyleSheet("background-color: white; color: black")
        elif self.statusColor.lbc_setpose == 1:
            self.bt_setpose.setStyleSheet("background-color: green; color: black")
        elif self.statusColor.lbc_setpose == 2:
            self.bt_setpose.setStyleSheet("background-color: blue; color: black")
        else:
            self.bt_setpose.setStyleSheet("background-color: red; color: white")

        # -- AGV toyo bit Read
        if (self.statusColor.lbc_toyoReadBit0 == 1):
            self.lbc_toyo_in0.setStyleSheet("background-color: blue; color: white;")
        elif (self.statusColor.lbc_toyoReadBit0 == 0):
            self.lbc_toyo_in0.setStyleSheet("background-color: white; color: black;")

        if (self.statusColor.lbc_toyoReadBit1 == 1):
            self.lbc_toyo_in1.setStyleSheet("background-color: blue; color: white;")
        elif (self.statusColor.lbc_toyoReadBit1 == 0):
            self.lbc_toyo_in1.setStyleSheet("background-color: white; color: black;")

        if (self.statusColor.lbc_toyoReadBit2 == 1):
            self.lbc_toyo_in2.setStyleSheet("background-color: blue; color: white;")
        elif (self.statusColor.lbc_toyoReadBit2 == 0):
            self.lbc_toyo_in2.setStyleSheet("background-color: white; color: black;")

        if (self.statusColor.lbc_toyoReadBit3 == 1):
            self.lbc_toyo_in3.setStyleSheet("background-color: blue; color: white;")
        elif (self.statusColor.lbc_toyoReadBit3 == 0):
            self.lbc_toyo_in3.setStyleSheet("background-color: white; color: black;")

        if (self.statusColor.lbc_toyoReadBit4 == 1):
            self.lbc_toyo_in4.setStyleSheet("background-color: blue; color: white;")
        elif (self.statusColor.lbc_toyoReadBit4 == 0):
            self.lbc_toyo_in4.setStyleSheet("background-color: white; color: black;")

        if (self.statusColor.lbc_toyoReadBit5 == 1):
            self.lbc_toyo_in5.setStyleSheet("background-color: blue; color: white;")
        elif (self.statusColor.lbc_toyoReadBit5 == 0):
            self.lbc_toyo_in5.setStyleSheet("background-color: white; color: black;")

        if (self.statusColor.lbc_toyoReadBit6 == 1):
            self.lbc_toyo_in6.setStyleSheet("background-color: blue; color: white;")
        elif (self.statusColor.lbc_toyoReadBit6 == 0):
            self.lbc_toyo_in6.setStyleSheet("background-color: white; color: black;")

        if (self.statusColor.lbc_toyoReadBit7 == 1):
            self.lbc_toyo_in7.setStyleSheet("background-color: blue; color: white;")
        elif (self.statusColor.lbc_toyoReadBit7 == 0):
            self.lbc_toyo_in7.setStyleSheet("background-color: white; color: black;")

    def controlShow_followMode(self):
        if (self.valueLable.modeRuning == self.modeRun_launch):
             self.modeRuning = self.modeRun_launch

        elif (self.valueLable.modeRuning == self.modeRun_byhand):           # Archie
            self.modeRuning = self.modeRun_byhand

        elif (self.valueLable.modeRuning == self.modeRun_auto):
            self.modeRuning = self.modeRun_auto
        else:
            self.modeRuning = self.modeRun_auto

        # if self.saveModeRuning != self.modeRuning:
        #     self.saveModeRuning = self.modeRuning
        #     self.status_cancelMission = 0
        #     self.isShow_setting = 0
        #     self.status_getInfo = 0

        if (self.modeRuning == self.modeRun_launch):
            self.fr_launch.show()
            self.fr_run.hide()
            self.show_launch()

        else:
            self.fr_run.show()
            self.fr_launch.hide()

            if (self.modeRuning == self.modeRun_auto):
                self.fr_setting.hide()
                if self.valueLable.status_show_nav == 1:
                    self.fr_agv_nav.show()
                    self.fr_agv_image.hide()

                    self.fr_infoModeAuto.show()
                    self.fr_auto.hide()
                    self.fr_handMode_move.hide()
                    self.fr_handMode_onof.hide()
                    self.fr_infoDevice.hide()
                    
                else:
                    self.fr_agv_nav.hide()
                    self.fr_agv_image.show()

                    if self.status_getInfo == 1:
                        self.fr_infoModeAuto.show()
                        self.fr_auto.hide()
                        self.fr_handMode_move.hide()
                        self.fr_handMode_onof.hide()
                        self.fr_infoDevice.hide()

                    else:
                        self.fr_auto.show()
                        self.fr_infoModeAuto.hide()
                        self.fr_handMode_move.hide()
                        self.fr_handMode_onof.hide()
                        self.fr_infoDevice.hide()

            elif (self.modeRuning == self.modeRun_byhand):
                # if self.status_cancelMission == 1:
                #     self.fr_password.show()
                #     self.fr_infoAGV.hide()
                # else:
                #     self.fr_password.hide()
                #     self.fr_infoAGV.show()
                #     self.fr_agv_image.show()

                self.fr_agv_nav.hide()
                self.fr_agv_image.show()

                if self.isShow_setting == 1 and self.status_show_setting == 1:
                    self.fr_setting.show()
                    self.fr_infoModeAuto.hide()
                    self.fr_auto.hide()
                    self.fr_handMode_move.hide()
                    self.fr_handMode_onof.hide()
                    self.fr_infoDevice.hide()

                    self.fr_password.hide()
                    self.fr_infoAGV.show()

                elif self.isShow_setting == 1 and self.status_show_setting == 0:
                    self.fr_password.show()
                    self.fr_infoAGV.hide()

                    if self.handMode == self.handMode1:
                        self.fr_handMode_move.show()
                        self.fr_handMode_onof.hide()
                        self.fr_infoModeAuto.hide()
                        self.fr_infoDevice.hide()
                        self.fr_auto.hide()

                    else:
                        self.fr_handMode_onof.show()
                        self.fr_handMode_move.hide()
                        self.fr_infoModeAuto.hide()
                        self.fr_infoDevice.hide()
                        self.fr_auto.hide()

                elif self.isShow_setting == 0:
                    self.fr_setting.hide()                
                    self.fr_password.hide()
                    self.fr_infoAGV.show()

                    if self.status_getInfo == 1:
                        # print("im here...")
                        self.fr_infoDevice.show()
                        self.fr_handMode_onof.hide()
                        self.fr_infoModeAuto.hide()
                        self.fr_handMode_move.hide()
                        self.fr_auto.hide()

                    else:
                        if self.handMode == self.handMode1:
                            self.fr_handMode_move.show()
                            self.fr_handMode_onof.hide()
                            self.fr_infoModeAuto.hide()
                            self.fr_infoDevice.hide()
                            self.fr_auto.hide()

                        else:
                            self.fr_handMode_onof.show()
                            self.fr_handMode_move.hide()
                            self.fr_infoModeAuto.hide()
                            self.fr_infoDevice.hide()
                            self.fr_auto.hide()

    def process_normal(self):
        self.set_dateTime()
        self.lbv_ip.setText(self.valueLable.lbv_ip)
        self.lbv_name_agv.setText(self.valueLable.lbv_name_agv)
        self.lbv_mac.setText(self.valueLable.lbv_mac)
        self.lbv_namePc.setText(self.valueLable.lbv_namePc)

        # --
        # self.TheCanvas.isUpdatePath = True                    
        # self.TheCanvas.makePath(self.valueLable.listPath)
        # self.TheCanvas.isUpdateAGV = True
        # self.TheCanvas.makeAGV(self.valueLable.lbv_coordinates_x, self.valueLable.lbv_coordinates_y, self.valueLable.lbv_coordinates_r)
        # self.fr_pathPlan.update()

        self.TheCanvas.update_Target(self.valueLable.typeTarget)
        self.TheCanvas.update_listTarget(self.valueLable.list_indexTarget)

        self.TheCanvas.isUpdatePath = True
        self.TheCanvas.makePath(self.valueLable.listPath, self.ratio_w, self.ratio_h)

        self.TheCanvas.isUpdateFullPath = True
        self.TheCanvas.makeFullPath(self.valueLable.listFullPath, self.ratio_w, self.ratio_h)

        self.TheCanvas.isUpdateAGV = True
        self.TheCanvas.makeAGV(self.valueLable.lbv_coordinates_x, self.valueLable.lbv_coordinates_y, self.valueLable.lbv_coordinates_r, self.ratio_w, self.ratio_h)
        
        self.fr_pathPlan.update()

    def process_slow(self):
        self.lbv_battery.setText(self.valueLable.lbv_battery)

    def process_fast(self):
        if self.valueLable.is_recvNNinfoRespond == 1:
            if self.valueLable.lbv_jobDoing != 0: # AGV đã ms khởi động xong
                if self.valueLable.lbv_jobDoing == 10:   # AGV chuyển sang chế độ tự động sau khi khởi động xong
                    self.bt_doNextPlan.setText("BẮT ĐẦU")
                    self.bt_doNextPlan.setStyleSheet("background-color: rgb(80,249,255);")
                    self.runOnce1 = 1

                elif self.valueLable.lbv_jobDoing == 6:    # Agv đang dừng do nhấn nút tạm dừng
                    self.bt_doNextPlan.setText("BẮT ĐẦU")
                    self.bt_doNextPlan.setStyleSheet("background-color: rgb(80,249,255);")
                    self.runOnce = 1
                    self.runOnce1 = 1

                elif self.valueLable.lbv_jobDoing == 8:    # AGV đang chờ lệnh mới
                    self.bt_doNextPlan.setText("BẮT ĐẦU")
                    self.bt_doNextPlan.setStyleSheet("background-color: rgb(80,249,255);")
                    # self.status_getInfo = 0
                    self.runOnce = 1
                    self.runOnce1 = 1

                elif self.valueLable.lbv_jobDoing == 20:   # AGV ở chế độ bằng tay
                    if self.runOnce1 == 1:
                        self.runOnce1 = 0
                        self.status_getInfo = 0 
                        self.runOnce = 1              

                else:
                    if self.valueLable.lbv_jobDoing != 30:
                        self.statusButton.bt_doNextPlan2 = 0

                    self.runOnce1 = 1
                    self.bt_doNextPlan.setText("TẠM DỪNG")
                    self.bt_doNextPlan.setStyleSheet("background-color: rgb(252, 175, 62);")
                    if self.runOnce == 1:
                        print("im here...")
                        # self.status_getInfo = 0
                        self.runOnce = 0

        # - cb of list speak
        self.statusButton.soundtype = self.cb_listSpeak.currentIndex()

        # - cb of list led
        self.statusButton.ledtype = self.cb_listLed.currentIndex()

        # - cb of list Toyo
        self.statusButton.toyobit = self.cb_listToyo.currentIndex()

        # - combo box
        if (self.valueLable.listError != self.valueLable.listError_pre):
            self.valueLable.listError_pre = self.valueLable.listError
            self.cb_status.clear()
            self.cb_status.addItems(self.valueLable.listError)
        
        # - address set pose
        if self.valueLable.isUpdateListIdSetpose == True:
            # print("Update combo box")
            self.valueLable.isUpdateListIdSetpose = False
            self.cb_listAddressSetPose.clear()
            # self.cb_listAddressSetPose.setEditable(True)
            self.cb_listAddressSetPose.addItems(self.valueLable.listIDSetpose)
            # line_edit = self.cb_listAddressSetPose.lineEdit()
            # line_edit.setAlignment(Qt.AlignCenter)
            # line_edit.setReadOnly(True)

            self.cb_listAddressSetPose2.clear()
            # self.cb_listAddressSetPose2.setEditable(True)
            self.cb_listAddressSetPose2.addItems(self.valueLable.listIDSetpose)
            # line_edit1 = self.cb_listAddressSetPose2.lineEdit()
            # line_edit1.setAlignment(Qt.AlignCenter)
            # line_edit1.setReadOnly(True)

        # -
        # if (self.valueLable.list_logError != self.valueLable.list_logError_pre):
        #     self.valueLable.list_logError_pre = self.valueLable.list_logError
        #     self.cb_logError.clear()
        #     lg = len(self.valueLable.list_logError) 
        #     for i in range(lg):
        #         self.cb_logError.addItem(self.valueLable.list_logError[i])

        # - 
        # self.statusButton.bt_getInfo = self.status_getInfo
        if self.isShow_setting == 1 and self.status_show_setting == 1:
            self.statusButton.bt_setting = 1
        else:
            self.statusButton.bt_setting = 0
        # self.statusButton.bt_setting = self.isShow_setting

        # # --
        self.set_labelValue()
        # # --
        self.set_labelColor()
        # # --
        self.controlShow_followMode()
        # -- 
        self.show_password()
        # -- 
        self.coorAverage_run()

        if (self.setting_status == 1):
            delta_t = rospy.Time.now() - self.timeSave_setting
            # -- Chi kich hoat khi dang o che do Bang Tay.
            if (delta_t.to_sec() > 1) and self.valueLable.modeRuning == 1:
                self.isShow_setting = 1
                self.password_data = ""
        else:
            self.timeSave_setting = rospy.Time.now()

        if self.statusButton.bt_getInfo == 1:
            delta_t = rospy.Time.now() - self.timeSave_getInfo
            if delta_t.to_sec() > 0.5:
                self.status_getInfo = 1

        else:
            self.timeSave_getInfo = rospy.Time.now()

        if self.statusButton.bt_cancelMission == 1:
            delta_t = rospy.Time.now() - self.timeSave_cancelMisson
            if delta_t.to_sec() > 0.5 and self.valueLable.modeRuning == self.modeRun_byhand:
                self.status_cancelMission = 1
        else:
            self.timeSave_cancelMisson = rospy.Time.now()

    def coorAverage_run(self):
        if (self.bt_coorAverage_status == 1):
            self.countTime_coorAverage += 1.
            self.total_x += self.robotPoseNow.position.x
            self.total_y += self.robotPoseNow.position.y
            euler = self.quaternion_to_euler(self.robotPoseNow.orientation)
            self.total_angle += degrees(euler)

        else:
            self.timeSave_coorAverage = rospy.Time.now()
            if (self.countTime_coorAverage > 4):
                d_x = self.total_x/self.countTime_coorAverage
                d_y = self.total_y/self.countTime_coorAverage
                d_a = self.total_angle/self.countTime_coorAverage
                d_degree = 0
                if d_a < 0:
                    d_degree = 360 + d_a
                else:
                    d_degree = d_a

                self.lbv_coorAverage_times.setText(str(self.countTime_coorAverage))
                self.lbv_coorAverage_x.setText(str(round(d_x, 3)))
                self.lbv_coorAverage_y.setText(str(round(d_y, 3)))
                self.lbv_coorAverage_r.setText(str(round(d_degree, 2)))

                if self.ck_linkCoor.isChecked() == 1:
                    self.valueLable.lbv_tryTarget_x = d_x
                    self.valueLable.lbv_tryTarget_y = d_y
                    self.valueLable.lbv_tryTarget_r = d_degree
                    self.lbv_tryTarget_x.setText(str(round(self.valueLable.lbv_tryTarget_x, 3)))
                    self.lbv_tryTarget_y.setText(str(round(self.valueLable.lbv_tryTarget_y, 3)))
                    self.lbv_tryTarget_r.setText(str(round(self.valueLable.lbv_tryTarget_r, 3)))

            self.countTime_coorAverage = 0
            self.total_x = 0.0
            self.total_y = 0.0
            self.total_angle = 0.0

    def show_launch(self):
        self.lbv_launhing.setText(self.valueLable.lbv_launhing)
        self.lbv_numberLaunch.setText(str(self.valueLable.lbv_numberLaunch))
        # --
        value = self.valueLable.percentLaunch
        if (value < 0):
            value = 0

        if (value > 100):
            value = 100

        self.pb_launch.setValue(value)
        # --
        # -- Port: RTC Board
        if (self.statusColor.lbc_port_rtc == 1):
            self.lbc_lh_rtc.setStyleSheet("background-color: blue; color: white")
        elif (self.statusColor.lbc_port_rtc == 0):
            self.lbc_lh_rtc.setStyleSheet("background-color: red; color: white;")

        # -- Port: Driver Left
        if (self.statusColor.lbc_port_driverLeft == 1):
            self.lbc_lh_driver_left.setStyleSheet("background-color: blue; color: white")
        elif (self.statusColor.lbc_port_driverLeft == 0):
            self.lbc_lh_driver_left.setStyleSheet("background-color: red; color: white;")

        # -- Port: Driver Right
        if (self.statusColor.lbc_port_driverRight == 1):
            self.lbc_lh_driver_right.setStyleSheet("background-color: blue; color: white")
        elif (self.statusColor.lbc_port_driverRight == 0):
            self.lbc_lh_driver_right.setStyleSheet("background-color: red; color: white;")

        # -- Port: Lidar
        if (self.statusColor.lbc_port_lidar == 1):
            self.lbc_lh_lidar.setStyleSheet("background-color: blue; color: white")
        elif (self.statusColor.lbc_port_lidar == 0):
            self.lbc_lh_lidar.setStyleSheet("background-color: red; color: white;")

    def show_password(self):
        data = ""
        lenght = len(self.password_data)

        for i in range(lenght):
            data += "*"
        
        self.lbv_pw_data.setText(data)

        if (self.password_data == self.password_right):
            self.bt_pw_agree.setEnabled(True)
        else:
            self.bt_pw_agree.setEnabled(False)

    def showPathPlan(self):
        pass

    def quaternion_to_euler(self, qua):
        quat = (qua.x, qua.y, qua.z, qua.w )
        a, b, euler = euler_from_quaternion(quat)
        return euler

    def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
        x = p2.x - p1.x
        y = p2.y - p1.y
        return sqrt(x*x + y*y)





