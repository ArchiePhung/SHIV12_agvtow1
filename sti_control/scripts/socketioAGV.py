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
from sti_msgs.msg import Navi_cmdRequest, PathInfo, StopPoint

# import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Point

from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from sti_msgs.msg import *
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees


class Communicate_socketIO():
    def __init__(self):
        rospy.init_node('sim_robot3', anonymous=False)
        self.rate = rospy.Rate(20)
        # -- 
        self.name_card = rospy.get_param("~name_card", "wlp0s20f3")
        self.name_card = "wlo2" # "wlp0s20f3"
        self.server_IP = rospy.get_param("~server_IP", '192.168.1.99')
        # self.server_IP = '127.0.0.1'
        self.server_port = rospy.get_param("~server_port", 4001)

        self.AGV_IP = rospy.get_param("~AGV_IP", '192.168.1.100')
        self.AGV_port = rospy.get_param("~AGV_port", 6000)
        self.AGV_mac = rospy.get_param("~AGV_mac", "6c:6a:77:df:90:6c")
        self.saveTimeStampServer = rospy.get_time()

        self.topicPose = rospy.get_param("~topicPose", "/robot_pose")
        self.topic_pubNaviCmd = rospy.get_param("~topic_pubNaviCmd", "/Navi_cmdRequest")
        # -
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_r = 0.0
        self.timeD = 1./8.
        self.saveTime = rospy.get_time()
        # -
        self.Navi_cmdPub = rospy.Publisher(self.topic_pubNaviCmd, Navi_cmdRequest, queue_size=50)
        self.Navi_cmdRequest = Navi_cmdRequest()

        self.NN_infoRequestPub = rospy.Publisher("/NN_infoRequest", NN_infoRequest, queue_size=50)
        self.NN_infoRequest = NN_infoRequest()	
        # # -
        # self.NN_infoRequestPub = rospy.Publisher("/NN_infoRequest", NN_infoRequest, queue_size=50)
        # self.server_requestInfor = NN_infoRequest()

        rospy.Subscriber(self.topicPose, PoseStamped, self.callback_poseRobot, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.poseStampedAGV = PoseStamped()
        self.theta_robotNow = 0.0

        # -- 
        self.straightLineMode = 1
        self.circleMode = 2
        self.quadraticBezierCurvesMode = 3

        # ---------
        self.saveTime_received = rospy.get_time()

    def AGVInfor_callback(self, data):
        self.AGV_information = data
        self.is_AGVInfor = 1

    def measureFreq_event(self):
        delta_t = rospy.get_time() - self.saveTime_received
        self.saveTime_received = rospy.get_time()
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
    
    # function callback
    def callback_poseRobot(self, data):
        self.poseStampedAGV = data
        self.poseRbMa = data.pose
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_robotNow = euler[2]

        self.robot_x = self.poseRbMa.position.x
        self.robot_y = self.poseRbMa.position.y
        self.robot_r = self.theta_robotNow
        self.robot_r = self.limitAngle(self.robot_r)

        self.is_pose_robot = True

    def checkLostDataNavi(self):
        dentaTime = rospy.get_time() - self.saveTimeStampServer
        if (dentaTime > 1.):
            return 1
        return 0

    def Navi_cmdAnalysis(self, data):
        # cmd
        self.Navi_cmdRequest.target_id = data['command']['target']['id']
        self.Navi_cmdRequest.target_x = data['command']['target']['x']
        self.Navi_cmdRequest.target_y = data['command']['target']['y']
        self.Navi_cmdRequest.target_z = data['command']['target']['r']
        self.Navi_cmdRequest.before_mission = data['command']['job']['before']
        self.Navi_cmdRequest.after_mission = data['command']['job']['after']
        self.Navi_cmdRequest.commandStop = data['command']['requirStop']
        # self.Navi_cmdRequest.enableStop = data['navigation']['position_stop']['enable']
        self.Navi_cmdRequest.enableStop = data['navigation']['position_stop']['enable']
        # navi
        # self.Navi_cmdRequest.command = data['navigation']['notification']
        self.Navi_cmdRequest.pointStopAvoid.pathID = data['navigation']['position_stop']['id']
        self.Navi_cmdRequest.pointStopAvoid.point.position.x = data['navigation']['position_stop']['x']
        self.Navi_cmdRequest.pointStopAvoid.point.position.y = data['navigation']['position_stop']['y']
        #-- list point
        paths = data['navigation']['path_global']['paths']
        point = data['navigation']['path_global']['points']
        listPath = []
        for i in range(len(point) - 1):
            pathInfo = PathInfo()
            pathInfo.pathID = paths[i]['id']
            pathInfo.typePath = paths[i]['type']
            #-- convert type path
            if (pathInfo.typePath == 0):
                pathInfo.typePath = self.straightLineMode
            elif (pathInfo.typePath == 1):
                pathInfo.typePath = self.quadraticBezierCurvesMode
            # -
            direct = paths[i]['direction_ab']
            if (paths[i]['direction_ba']['start'] == point[i]['id']):
                direct = paths[i]['direction_ba']
            # - 
            pathInfo.direction = 1
            if (not direct['forward'] and direct['backward']):
                pathInfo.direction = 2
            # - 
            pathInfo.velocity = direct['speed']
            pathInfo.movableZone = direct['road_width']
            # - 
            pointOne = Pose()
            pointOne.position.x = point[i]['x']
            pointOne.position.y = point[i]['y']
            pathInfo.pointOne = pointOne

            pointSecond = Pose()
            pointSecond.position.x = point[i+1]['x']
            pointSecond.position.y = point[i+1]['y']
            pathInfo.pointSecond = pointSecond

            pointMid = Pose()
            pointMid.position.x = paths[i]['q']['x']
            pointMid.position.y = paths[i]['q']['y']
            pathInfo.pointMid = pointMid
            # -
            listPath.append(pathInfo)
            # -  
        
        self.Navi_cmdRequest.pathInfo = listPath
        # publish
        self.Navi_cmdPub.publish(self.Navi_cmdRequest)

    def run(self):
        if (self.checkLostDataNavi()):
            print("lost data navi")
        # self.Navi_cmdPub.publish(self.Navi_cmdRequest)
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
        print('I received Request form Server!')

        data_json = json.loads(data)
        print (data_json['mac'])
        if data_json['mac'] == myObject.AGV_mac:
            # print ("Server request: Info to Me!")
            myObject.measureFreq_event()

            x = round(myObject.robot_x, 3)
            y = round(myObject.robot_y, 3)
            r = round(myObject.robot_r, 3)
            battery = round(random.uniform(23.0, 25.5), 1)
            status = round(random.uniform(0, 2), 0)
            mode = round(random.uniform(0, 2), 0)
            # print ("x: ", x)
            # x = round(myObject.AGV_information.x, 3)
            # y = round(myObject.AGV_information.y, 3)
            # r = round(myObject.AGV_information.z, 3)
            # status = myObject.AGV_information.status
            # battery = myObject.AGV_information.battery
            # mode = myObject.AGV_information.mode
            listErrors = [] # myObject.AGV_information.listError

            data_send = {"id": data_json['id'], "name": data_json['name'], "mac": data_json['mac'], "mode": mode, "status": 1, "x": x, "y": y, "r": r, "status": status, "battery": battery, "listErrors": listErrors}
            my_socketIO.emit("Robot-respond-info", json.dumps(data_send, indent = 4))

    @my_socketIO.on('Server-send-agv-cmd')
    def on_message(data):
        data_json = json.loads(data)
        # print(data_json)
        myObject.saveTimeStampServer = rospy.get_time()
        myObject.Navi_cmdAnalysis(data_json)
        # print ("Server send: Command to Me!")

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


#--
# "command": {
#     "target": {
#         "id": 0,
#         "x": 0,
#         "y": 0,
#         "r": 0
#     },
#     "job": {
#         "before": 0,
#         "after": 0
#     },
#     "requirStop": 0,
# },
# "navigation": {
#     "onRoad": 0,
#     "position_stop": {
#        "enable": 0,
#        "id": 0, // - ID path
#        "x": 0,
#        "y": 0,
#        "nameAGV": ""
#     },
#     "path_global": {
#         "paths": [], // - path_object()
#         "points": [] // - point_object()
#     }, 
#     "path_local": [],  // - {"x", "y"}
#     "points": [],
#     "stop_avoiding": {
#         "id": 0,
#         "name": ""
#     },
#     "notification": ""
# }

# class point_object {
# 	constructor() {
# 		this.id = 0
# 		this.enableDo = 0
# 		this.codeDo = ""
# 		this.x = 0.0
# 		this.y = 0.0
# 		this.tag = ""
# 		this.name = ""
# 	}	
# }

# class path_object {
# 	constructor() {
# 		this.id = 0
# 		this.name = 0
# 		this.enable = 0
# 		this.type = 0
# 		this.a = 0
# 		this.b = 0
# 		this.direction_ab = {
# 			"enable": 0,
# 		 	"speed": 0,
# 			"safety": 0,
# 			"forward": 1,
# 			"backward": 1,
# 			"angle_follow": -1,
# 			"road_width": -1,
# 			"angle_error": -1
# 		}
# 		this.direction_ba = {
# 			"enable": 0,
# 		 	"speed": 0,
# 			"safety": 0,
# 			"forward": 1,
# 			"backward": 1,
# 			"angle_follow": -1,
# 			"road_width": -1,
# 			"angle_error": -1
# 		}
# 		this.q = {
# 			"x": 0.0,
# 			"y": 0.0
# 		}
# 	}
# }