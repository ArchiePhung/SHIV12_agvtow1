#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#----- Info ------
# Navigation program for AGV NAV STI
# Date: 25/9
# Author: PhucHoang 

# ---------- Update ------------
# Hoang sua code ngay 25/07/2023
# Hoang sua code cho AGV keo demo Shiv12 22/01/2024
# Hoang sua code cho AGV keo SHIV12 30/5/2023

import roslib
import copy
import sys
import time
import signal
import rospy
from std_msgs.msg import String, Int8
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point
from sti_msgs.msg import PathInfo, PointRequestMove, ListPointRequestMove, LineRequestMove, HC_info, Status_goal_control, Zone_lidar_2head, StopPoint
import numpy as np
from math import sqrt, pow, atan, fabs, cos, sin, acos, degrees, radians, atan2
from math import pi as PI
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
import threading


class Point():
  def __init__(self, _x=0, _y=0):
    self.x = _x
    self.y = _y
    self.id = -1

class InfoPathFollowing():
    def __init__(self):
        self.pathID = 0.
        self.typePath = 0
        self.direction = 0
        self.indexInListpath = 0.
        self.velocity = 0.
        self.radius = 0.
        self.movableZone = 0.
        self.X = 0.
        self.Y = 0.
        self.fieldSafety = 0

class StraightLine():
    def __init__(self, _pointOne=Point(), _pointSecond=Point()):
        self.pointOne = _pointOne
        self.pointSecond = _pointSecond
        
        self.a = self.pointOne.y - self.pointSecond.y
        self.b = self.pointSecond.x - self.pointOne.x
        self.c = -self.pointOne.x*self.a - self.pointOne.y*self.b
        # print(self.a,self.b,self.c)
        self.dis = sqrt(self.a*self.a + self.b*self.b)

    def getQuaternion(self):
        euler = 0.0
        if self.b == 0:
            if self.a < 0:
                euler = PI/2.0
            elif self.a > 0:
                euler = -PI/2.0
        elif self.a == 0:
            if -self.b < 0:
                euler = 0.0
            elif -self.b > 0:
                euler = PI

        else:
            euler = acos(self.b/sqrt(self.b*self.b + self.a*self.a))
            if -self.a/self.b > 0:
                if fabs(euler) > PI/2:
                    euler = -euler
                else:
                    euler = euler
            else:
                if fabs(euler) > PI/2:
                    euler = euler
                else:
                    euler = -euler
                    
        return quaternion_from_euler(0.0, 0.0, euler)

    def checkPointInLine(self, _pointX, _pointY, X_reference, Y_reference):
        # loai nghiem bang vector
        vector_qd_x = self.pointOne.x - self.pointSecond.x
        vector_qd_y = self.pointOne.y - self.pointSecond.y

        vector_point_x = X_reference - _pointX
        vector_point_y = Y_reference - _pointY

        if (vector_qd_y*vector_point_y > 0 or vector_qd_x*vector_point_x > 0):
            return True

        # if vector_qd_x == 0.0:
        #     if vector_qd_y*vector_point_y > 0.0:
        #         return True
            
        # elif vector_qd_y == 0.0:
        #     if vector_qd_x*vector_point_x > 0.0:
        #         return True

        # else:
        #     v_a = vector_qd_x/vector_point_x
        #     v_b = vector_qd_y/vector_point_y
        #     if v_a*v_b > 0.0 and v_a > 0.0:
        #         return True

        return False
    
    def find_HC(self, _poseX, _poseY):
        X_n = Y_n = 0.
        if _poseX == self.pointOne.x and _poseY == self.pointOne.y :
            X_n = self.pointOne.x
            Y_n = self.pointOne.y

        else:
            a_hc = self.b
            b_hc = -self.a
            c_hc = -_poseX*a_hc - _poseY*b_hc
            # print(_poseX, _poseY, self.a, self.b, self.c, a_hc, b_hc, c_hc)

            X_n = ((c_hc*self.b)-(self.c*b_hc))/((self.a*b_hc)-(self.b*a_hc))
            Y_n = ((c_hc*self.a)-(self.c*a_hc))/((a_hc*self.b)-(b_hc*self.a))

        return X_n, Y_n
    
    def findGoalWithLookAhead(self, _poseX, _poseY, _L):
        X_g1 = Y_g1 = X_g2 = Y_g2 = 0.0
        Xhc, Yhc = self.find_HC(_poseX, _poseY)

        if round(self.b, 5) == 0.0:
            X_g1 = X_g2 = -self.c/self.a
            Y_g1 = -sqrt(_L*_L - (X_g1 - Xhc)*(X_g1 - Xhc)) + Yhc
            Y_g2 = sqrt(_L*_L - (X_g2 - Xhc)*(X_g2 - Xhc)) + Yhc
            
        else:
            la = (1.0 + (self.a/self.b)*(self.a/self.b))
            lb = -2.0*(Xhc - (self.a/self.b)*((self.c/self.b) + Yhc))
            lc = Xhc*Xhc + ((self.c/self.b) + Yhc)*((self.c/self.b) + Yhc) - _L*_L
            denlta = lb*lb - 4.0*la*lc

            X_g1 = (-lb + sqrt(denlta))/(2.0*la)
            X_g2 = (-lb - sqrt(denlta))/(2.0*la)

            Y_g1 = (-self.c - self.a*X_g1)/self.b
            Y_g2 = (-self.c - self.a*X_g2)/self.b

        if self.checkPointInLine(X_g1, Y_g1, Xhc, Yhc):
            return X_g1, Y_g1
        else:
            return X_g2, Y_g2
        

class GoalControl(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        self.saveTimeMainTheard = rospy.get_time()
        self.startLoop = False
        self.rate = rospy.Rate(30)

        # topic
        self.topic_CmdVel = rospy.get_param('~topic_CmdVel', 'cmd_vel')
        self.topic_pose = rospy.get_param('~topic_pose', '/robotPose_lidarLOC')
        self.topic_subListP = rospy.get_param('~topic_subListP', 'list_pointRequestMove')
        self.topic_pubRes = rospy.get_param('~topic_pubRes', 'move_respond')

        # gioi han van toc
        self.min_angularVelocity = rospy.get_param('~min_angularVelocity', 0.01)
        self.max_angularVelocity = rospy.get_param('~min_angularVelocity', 1.)

        self.max_linearVelocity = rospy.get_param('~max_linearVelocity', 1.1) #0.1
        self.min_linearVelocity = rospy.get_param('~min_linearVelocity', 0.05) # 0.008

        self.max_lookahead = rospy.get_param('~max_lookahead', 2.05)
        self.min_lookahead = rospy.get_param('~min_lookahead', 0.2)
        self.lookahead_ratio = rospy.get_param('~lookahead_ratio', 8.0)

        # sai so
        self.toleranceByX_PointFinish = rospy.get_param('~toleranceByX_PointFinish', 0.01)
        self.toleranceByY_PointFinish = rospy.get_param('~toleranceByY_PointFinish', 0.01)
        self.toleranceByX_PointStop = rospy.get_param('~toleranceByX_PointStop', 0.01)
        self.toleranceByY_PointStop = rospy.get_param('~toleranceByY_PointStop', 0.01)

        rospy.Subscriber(self.topic_pose, PoseStamped, self.callback_poseRobot, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.poseStampedAGV = PoseStamped()
        self.theta_robotNow = 0.0

        # rospy.Subscriber('/move_request', LineRequestMove, self.callback_moveRequest)
        # self.dataRequestMove = LineRequestMove() 
        # self.is_moveRequest = False

        rospy.Subscriber(self.topic_subListP, ListPointRequestMove, self.callback_listPointRequestMove, queue_size = 1)
        self.datalistPointRequestMove = ListPointRequestMove() 
        self.startPoint = InfoPathFollowing()
        self.finishPoint = InfoPathFollowing()
        self.is_listPointRequestMove = False

        rospy.Subscriber("/safety_zone", Zone_lidar_2head, self.safety_callback)
        self.respSafety = Zone_lidar_2head() 
        self.is_safetyZone = False
        self.timeCheckSafety = rospy.get_time()
        self.lostSafety = False
        self.statusZone = 0

        rospy.Subscriber("/request_move", LineRequestMove, self.callback_moveRequest)
        self.enb = 0
        self.is_moveRequest = False

        # Pub topic
        self.pub_cmd_vel = rospy.Publisher(self.topic_CmdVel, Twist, queue_size=20)
        self.time_tr = rospy.get_time()
        self.rate_pubVel = 30

        self.pubMarker = rospy.Publisher('/visualization_markerPoint', Marker, queue_size=10)

        self.pubFieldSafety = rospy.Publisher('/fieldSelect', Int8, queue_size=10)
        self.fieldRequest = Int8()

        rospy.on_shutdown(self.fnShutDown)

        self.pubMoveRespond = rospy.Publisher(self.topic_pubRes, Status_goal_control, queue_size=10) 

        self.datalistPointRequestMoveNow = ListPointRequestMove()
        self.coordinate_unknown = 500.0
        self.target_x = self.coordinate_unknown
        self.target_y = self.coordinate_unknown
        self.target_z = 0.0

        self.process = 0
        self.stepCheckStart = 0
        self.angleLineSatrt = 0.

        self.min_vel = 0.05
        self.min_velFinish = 0.04
        self.min_rol = 0.1
        self.min_rolFinish = 0.15

        self.max_vel = 0.07
        self.max_rol = 0.3

        self.xFollow = 0.
        self.yFollow = 0.
        self.velFollow = 0.
        self.lineIDFollow = 0
        self.typeLineFollow = 0

        self.distDeceleration = 0.3
        self.disgetLocation = 0.15

        self.stepContourFollow = 0
        self.curr_velocity = 0.

        # self.listVel =       [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.9, 1.0, 1.1]
        # self.numTable = len(self.listVel)
        # self.listLookAhead = [0.2, 0.4, 0.55, 0.65, 0.82, 0.95, 1.03, 1.2, 1.28, 1.35, 1.45, 1.55, 1.75, 1.8, 1.85, 1.9, 2.05, 2.15]

        self.listVel = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.9, 1.0, 1.1]
        self.numTable = len(self.listVel)
        self.listLookAhead = [0.2, 0.36, 0.4, 0.46, 0.59, 0.68, 0.77, 0.87, 0.97, 1.07, 1.17, 1.35, 1.45, 1.50, 1.6, 1.7, 1.8, 1.9, 2.05] # old 18/07/2024
        # self.listLookAhead = [0.2, 0.36, 0.4, 0.46, 0.6, 0.75, 0.85, 0.95, 1.05, 1.1, 1.19, 1.35, 1.45, 1.50, 1.6, 1.7, 1.8, 1.9, 2.05]

        self.infoPathFollow = InfoPathFollowing()
        self.velFollowOutOfRange = 0.15

        self.cmdVel = 0.
        self.flagVelChange = True
        self.statusVel = 0. # 0: khong thay doi 1: dang tang toc, 2: dang giam toc
        self.velSt = 0.

        self.saveTimeVel = rospy.get_time()
        self.flagFollowPointStop = False
        self.flagCheckAngle = False

        self.infoGoalNearest = InfoPathFollowing()
        self.disRbToGoalNearest = 0.

        self.maxDisBD = 0.

        self.stepFollowGoalStop = 0
        self.STTarget = [0,0,0]

        self.decelerationRatio = 0.5
        self.disDeceleration = 0.

        self.saveAngle = 0.

        self.nextPointIsStopGoal = False
        self.infoGoalStop = StopPoint()

        self.stepMoveToPoint = 0
        self.saveSTL = StraightLine()

        self.stepMoveForward = 0
        self.XStart = 0.
        self.YStart = 0.

        self.pointStopAvoid = Point()

        self.directFollow = 0
        self.directForward = 1
        self.directReverse = 2

        # go straight pallet
        self.distanceGoStraight = 0.
        self.distanceGoStraightMax = 2.5

        # bien status 
        self.statusAGV = 0
        self.errorAGV = 0
        self.sSafety_AGV = 0
        self.completed_simple = 0      # bao da den dich.
        self.completed_backward = 0     # bao da den aruco.
        self.completed_all = False
        self.completed_reset = False
        self.end_of_list = False

        self.saveStartPoint = InfoPathFollowing()
        self.saveFinishPoint = InfoPathFollowing()

        #-- 
        self.isDecelerationObstacles = 0
        self.saveVelWhenDecObs = 0.
        self.maxVelZone3 = 0.2
        self.maxVelZone2 = 0.1

        # -- sai so duong dan
        self.path_error = 0.

    def callback_moveRequest(self, data):
        self.enb = data.enable

    def callback_poseRobot(self, data):
        self.poseStampedAGV = data
        self.poseRbMa = data.pose
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_robotNow = euler[2]

        self.is_pose_robot = True

    
    def callback_listPointRequestMove(self, data):
        self.datalistPointRequestMove = data
        try:
            self.startPoint.X = self.datalistPointRequestMove.infoPoint[0].pose.position.x
            self.startPoint.Y = self.datalistPointRequestMove.infoPoint[0].pose.position.y
            self.startPoint.indexInListpath = 0
            self.startPoint.movableZone = self.datalistPointRequestMove.infoPoint[0].movableZone
            self.startPoint.pathID = self.datalistPointRequestMove.infoPoint[0].pathID
            self.startPoint.typePath = self.datalistPointRequestMove.infoPoint[0].typePath
            self.startPoint.velocity = self.datalistPointRequestMove.infoPoint[0].velocity
            self.startPoint.fieldSafety = self.datalistPointRequestMove.infoPoint[0].fieldSafety

            self.finishPoint.X = self.datalistPointRequestMove.infoPoint[-1].pose.position.x
            self.finishPoint.Y = self.datalistPointRequestMove.infoPoint[-1].pose.position.y
            self.finishPoint.indexInListpath = len(self.datalistPointRequestMove.infoPoint) - 1
            self.finishPoint.movableZone = self.datalistPointRequestMove.infoPoint[-1].movableZone
            self.finishPoint.pathID = self.datalistPointRequestMove.infoPoint[-1].pathID
            self.finishPoint.typePath = self.datalistPointRequestMove.infoPoint[-1].typePath
            self.finishPoint.velocity = self.datalistPointRequestMove.infoPoint[-1].velocity
            self.finishPoint.fieldSafety = self.datalistPointRequestMove.infoPoint[-1].fieldSafety
        except:
            pass
        self.is_listPointRequestMove = True


    def safety_callback(self, data):
        self.respSafety = data
        self.is_safetyZone = True
        self.lostSafety = False
        self.timeCheckSafety = rospy.get_time()
    
    # check lost hc
    def checkLostSafety(self):
        if self.is_safetyZone:
            dentalTime = rospy.get_time() - self.timeCheckSafety
            if dentalTime > 1.:
                self.lostSafety = True
                return 1
        return 0 

    #check lost theard
    def checkLostMainTheard(self):
        if self.startLoop:
            dentalTime = rospy.get_time() - self.saveTimeMainTheard
            if dentalTime > 3.:
                return 1
        return 0

    # stop
    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist())

    def stop(self):
        self.curr_velocity = 0.
        self.statusVel = 0
        self.velSt = 0.
        self.saveTimeVel = rospy.get_time()
        for i in range(2):
            self.pub_cmd_vel.publish(Twist())


    # function pub
    def pub_cmdVel(self, twist , rate):
        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass

    def pubMakerPointFollow(self, x, y):
        # Create rviz marker message
        marker = Marker()
        marker.header.frame_id = "frame_map_nav350"
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'pointfollow'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = -2.45
        marker.pose.orientation.w = 1.
        marker.color.r = 1.0
        marker.color.g = 0.
        marker.color.b = 0.
        marker.color.a = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        self.pubMarker.publish(marker)

    def Meaning(self, stt, war):
        mess_step = {
            0: 'Step: Wait All data',
            1: 'Step: Select Mode',
            12: 'Kiem tra vi tri',
            13: 'Di chuyen den vi tri finish',
            14: 'Di chuyen den vi tri stop point',
            15: 'Di chuyen nhap vao duong dan',
            16: 'Follow duong dan',
            17: 'Dap ung goc cuoi',
            18: 'Hoan thanh - Doi reset :v',
            19: 'Cap nhat target',
            20: 'Phan chia proces'
        }
        mess_warn = {
            0: 'AGV di chuyen binh thuong :)',
            1: 'AGV gap vat can :(',
            2: 'AGV gap loi chuong trinh',
            3: 'AGV ra khoi duong dan'
        }
        mess = mess_step.get(stt, 'UNK') + ' | ' + mess_warn.get(war, 'UNK')
        return mess 
    
    def pub_status(self, target_x, target_y, target_z, misson, process_now, error, safety, complete_misson , id, _path_error, _typePath_follow):
        status = Status_goal_control()
        status.target_x = target_x
        status.target_y = target_y
        status.target_z = target_z
        status.misson = misson
        status.status_now = process_now
        status.ID_follow = int(id)
        status.typePath_follow = _typePath_follow
        status.error = error        # 0 (binh thuong) | 1 (AGV ra khỏi đường dẫn) | 2 (lỗi hệ thống) 
        status.safety = safety      # 0 (binh thuong) | 1 (có vật cản)
        status.meaning = self.Meaning(process_now, error)
        status.path_error = _path_error
        status.complete_misson = complete_misson
        self.pubMoveRespond.publish(status)

    # Function 
    def funcalptduongthang(self, X_s, Y_s, X_f, Y_f):
        _a = Y_s - Y_f
        _b = X_f - X_s
        _c = -X_s*_a -Y_s*_b
        return _a, _b, _c

    def SIGN(self, num):
        if num > 0:
            return 1
        elif num < 0:
            return -1
        return 0
    
    def constrain(self, value_in, value_min, value_max):
        value_out = 0.0
        if value_in < value_min:
            value_out = value_min
        elif value_in > value_max:
            value_out = value_max
        else:
            value_out = value_in
        return value_out
    
    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)
    
    # Function navigaton
    def rotary_around(self, dentalAngle, angular_velocity, permission_tolerance):
        twist = Twist()
        if fabs(dentalAngle) > permission_tolerance:
            if dentalAngle > 0.: # quay trai
                twist.angular.z = angular_velocity
            else:
                twist.angular.z = -angular_velocity
            self.pub_cmdVel(twist, self.rate_pubVel)
            
        else:
            self.stop()
            return 1
        
        return 0
    
    def findGoalNearest(self, xRb, yRb, datalistPointRequestMoveNow):
        index = -1
        min = float('inf')
        numpoint = len(datalistPointRequestMoveNow.infoPoint)
        self.path_error = 0.
        for i in range(numpoint - 1, -1, -1):
            x = datalistPointRequestMoveNow.infoPoint[i].pose.position.x
            y = datalistPointRequestMoveNow.infoPoint[i].pose.position.y
            # vel = self.dataPath.info[i].velocity
            dis = self.calculate_distance(x, y, xRb, yRb)

            if dis <= min:
                min = dis
                index = i
                self.path_error = dis
                
        infoPoint = InfoPathFollowing()
        if index != -1:  
            infoPoint.indexInListpath = index
            infoPoint.pathID = datalistPointRequestMoveNow.infoPoint[index].pathID
            # vel = (datalistPointRequestMoveNow.infoPoint[index].velocity/100.)*self.max_linearVelocity
            vel = datalistPointRequestMoveNow.infoPoint[index].velocity
            infoPoint.velocity = self.constrain(vel, self.min_linearVelocity, self.max_linearVelocity)
            infoPoint.typePath = datalistPointRequestMoveNow.infoPoint[index].typePath
            infoPoint.movableZone = datalistPointRequestMoveNow.infoPoint[index].movableZone
            infoPoint.fieldSafety = datalistPointRequestMoveNow.infoPoint[index].fieldSafety
            infoPoint.X = datalistPointRequestMoveNow.infoPoint[index].pose.position.x
            infoPoint.Y = datalistPointRequestMoveNow.infoPoint[index].pose.position.y
            # print('dis_rb_to_path: ', min)

        return infoPoint, min, numpoint
    
    def convert_relative_coordinates(self, X_cv, Y_cv):
        angle = -self.theta_robotNow
        _X_cv = (X_cv - self.poseRbMa.position.x)*cos(angle) - (Y_cv - self.poseRbMa.position.y)*sin(angle)
        _Y_cv = (X_cv - self.poseRbMa.position.x)*sin(angle) + (Y_cv - self.poseRbMa.position.y)*cos(angle)
        return _X_cv, _Y_cv

    def control_navigation(self, X_point_goal, Y_point_goal, vel_x): # 1 goal phia truoc | -1 goal phia sau
        vel_th = 0.0
        xGoal = X_point_goal
        yGoal = Y_point_goal

        if fabs(yGoal) <= 0.005:
            vel_th = 0.
            
        else:
            l = (xGoal*xGoal) + (yGoal*yGoal)
            r = l/(2*fabs(yGoal))
            # print(vel_x, r, l)
            vel = fabs(vel_x)/r
            if yGoal > 0:
                vel_th = vel*self.SIGN(vel_x)
            else:
                vel_th = -vel*self.SIGN(vel_x)

        return vel_th
    
    def findVelByLookAhead(self, l):
        a = 0.
        b = 0.
        if l >= self.max_lookahead:
            l = self.max_lookahead
        elif l <= self.min_lookahead:
            l = self.min_lookahead

        for i in range(self.numTable -1 ):
            if l >= self.listLookAhead[i] and l <= self.listLookAhead[i+1]:
                a = (self.listVel[i] - self.listVel[i+1])/(self.listLookAhead[i] - self.listLookAhead[i+1])
                b = self.listVel[i] - a*self.listLookAhead[i]

        return a*l + b
    
    def findLookAheadByVel(self, curr_velocity):
        a = 0.
        b = 0.
        if curr_velocity >= self.max_linearVelocity:
            curr_velocity = self.max_linearVelocity
        elif curr_velocity <= self.min_linearVelocity:
            curr_velocity = self.min_linearVelocity

        for i in range(self.numTable -1 ):
            if curr_velocity >= self.listVel[i] and curr_velocity <= self.listVel[i+1]:
                a = (self.listLookAhead[i] - self.listLookAhead[i+1])/(self.listVel[i] - self.listVel[i+1])
                b = self.listLookAhead[i] - a*self.listVel[i]

        return a*curr_velocity + b
    
    def angleStraightLine(self, XpointA, YpointA, XpointB, YpointB): # -- Angle Line Point A to Point B. | Point()
        a = YpointA - YpointB
        b = XpointB - XpointA
        ang = 0.0
        if b == 0:
            if a < 0:
                ang = PI/2.0
            elif a > 0:
                ang = -PI/2.0

        elif a == 0:
            if -b < 0:
                ang = 0.0
            elif -b > 0:
                ang = PI

        else:
            ang = acos(b/sqrt(b*b + a*a))
            if -a/b > 0:
                if fabs(ang) > PI/2:
                    ang = -ang
                else:
                    ang = ang
            else:
                if fabs(ang) > PI/2:
                    ang = ang
                else:
                    ang = -ang
        return ang
    
    def angleBetweenRobotAndPath(self, _quat):
        roll, pitch, yaw = euler_from_quaternion(_quat)
        dtAngle = yaw - self.theta_robotNow
        if fabs(dtAngle) >= PI:
            dtAngleTG = (2*PI - fabs(dtAngle))
            if dtAngle > 0:
                dtAngle = -dtAngleTG
            else:
                dtAngle = dtAngleTG

        return dtAngle
    
    def angleBetweenRobotAndPath2(self, _angle):
        dtAngle = _angle - self.theta_robotNow
        if fabs(dtAngle) >= PI:
            dtAngleTG = (2*PI - fabs(dtAngle))
            if dtAngle > 0:
                dtAngle = -dtAngleTG
            else:
                dtAngle = dtAngleTG

        return dtAngle
    
    def funcDecelerationByTime(self, denlta_time, time_s, v_s, v_f):
        denlta_time_now = rospy.get_time() - time_s
        a = (v_f-v_s)/denlta_time

    def funcDecelerationByAcc(self, time_s, v_s, v_f, a):
        denlta_time_now = rospy.get_time() - time_s
        v_re = v_s + a*denlta_time_now
        if a > 0.:
            if v_re >= v_f:
                v_re = v_f
        else:
            if v_re <= v_f:
                v_re = v_f

        return v_re

    def getVeloctity(self, velCmd, _statusVel):
        if _statusVel == 0:
            return velCmd
        elif _statusVel == 1:
            return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, velCmd, 0.025) # 0.02
        elif _statusVel == 2:
            return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, velCmd, -0.15)  #-0.15
        else:
            return 0.

        # if self.directFollow == self.directForward and (self.respSafety.zone_ahead == 1 or self.lostSafety):
        #     self.statusVel = 0
        #     print('not safety')
        #     return 0.
        # if _safety != 0: # trang thai giam toc do vat can
        #     if _safety == 1:
        #         self.statusVel = 0
        #         self.stop()
        #         return 0.
        #     elif _safety == 2: 
        #         self.statusVel = 0
        #         return self.infoPathFollow.velocity*(1./3.)
        #     else:
        #         self.statusVel = 0
        #         return self.infoPathFollow.velocity*(2./3.)
            
        # else:      
            
            
    def infoPointPathByIndex(self, _index, datalistPointRequestMoveNow):
        infoPoint = InfoPathFollowing()
        infoPoint.indexInListpath = _index
        infoPoint.pathID = self.datalistPointRequestMove.infoPoint[_index].pathID
        # vel = (self.datalistPointRequestMove.infoPoint[_index].velocity/100.)*self.max_linearVelocity
        vel = datalistPointRequestMoveNow.infoPoint[_index].velocity
        infoPoint.velocity = self.constrain(vel, self.min_linearVelocity, self.max_linearVelocity)
        infoPoint.typePath = self.datalistPointRequestMove.infoPoint[_index].typePath
        infoPoint.direction = self.datalistPointRequestMove.infoPoint[_index].direction
        infoPoint.movableZone = self.datalistPointRequestMove.infoPoint[_index].movableZone
        infoPoint.fieldSafety = self.datalistPointRequestMove.infoPoint[_index].fieldSafety
        infoPoint.X = self.datalistPointRequestMove.infoPoint[_index].pose.position.x
        infoPoint.Y = self.datalistPointRequestMove.infoPoint[_index].pose.position.y

        return infoPoint
    
    def moveIntoPath(self, datalistPointRequestMoveNow):
        indexSelect = -1
        lookahead = self.findLookAheadByVel(self.velFollowOutOfRange)
        self.infoGoalNearest, self.disRbToGoalNearest, numpoint = self.findGoalNearest(self.poseRbMa.position.x, self.poseRbMa.position.y, datalistPointRequestMoveNow)
        numStart = self.infoGoalNearest.indexInListpath
        numStop = numpoint
        self.sSafety_AGV = 0
        for i in range(numStart, numStop, 1):  # can kiem tra lai ------------------------------#####--------------------------------------
            x = datalistPointRequestMoveNow.infoPoint[i].pose.position.x
            y = datalistPointRequestMoveNow.infoPoint[i].pose.position.y
            pathID = datalistPointRequestMoveNow.infoPoint[i].pathID
            # vel = self.dataPath.info[i].velocity
            dis = self.calculate_distance(x, y, self.infoGoalNearest.X, self.infoGoalNearest.Y)
            if dis >= lookahead:
                indexSelect = i
                break

        self.infoPathFollow = self.infoPointPathByIndex(indexSelect, datalistPointRequestMoveNow)
        self.directFollow = self.infoPathFollow.direction

        self.infoGoalNearest, self.disRbToGoalNearest, numpoint = self.findGoalNearest(self.poseRbMa.position.x, self.poseRbMa.position.y, datalistPointRequestMoveNow)
        angleLineStart = 0.
        if self.directFollow == self.directForward:
            angleLineStart = self.angleStraightLine(self.infoGoalNearest.X, self.infoGoalNearest.Y, self.infoPathFollow.X, self.infoPathFollow.Y)
        elif self.directFollow == self.directReverse: 
            angleLineStart = self.angleStraightLine(self.infoPathFollow.X, self.infoPathFollow.Y, self.infoGoalNearest.X, self.infoGoalNearest.Y)

        angleBW = self.angleBetweenRobotAndPath2(angleLineStart)

        if self.stepCheckStart == 0:
            if self.directFollow == self.directForward:
                self.saveAngle = self.angleStraightLine(self.poseRbMa.position.x, self.poseRbMa.position.y, self.infoPathFollow.X, self.infoPathFollow.Y)
            elif self.directFollow == self.directReverse: 
                self.saveAngle = self.angleStraightLine(self.infoPathFollow.X, self.infoPathFollow.Y, self.poseRbMa.position.x, self.poseRbMa.position.y)
            self.stepCheckStart = 1

        if self.stepCheckStart == 1:
            angle = self.angleBetweenRobotAndPath2(self.saveAngle)
            if self.rotary_around(angle, 0.15, radians(2.)): # kiem tra goc AGV voi quy dao
                self.stepCheckStart = 2

        if self.stepCheckStart == 2:
            if self.disRbToGoalNearest <= 0.02  and fabs(angleBW) <= 10.*PI/180.: # dieu kien de xac nhan da vao duong dan
                self.stepCheckStart = 0
                return 1
            
            else:
                if self.directFollow == self.directForward and (self.respSafety.zone_ahead == 1 or self.lostSafety):
                    self.stop()
                    self.sSafety_AGV = 1
                    print('not safety')

                else:
                    twist = Twist()
                    velX = self.velFollowOutOfRange
                    self.curr_velocity = velX
                    xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X,self.infoPathFollow.Y)
                    directVel = velX if self.directFollow == self.directForward else -velX
                    velAng = self.control_navigation(xCVFollow, yCVFollow, directVel)
                    twist.linear.x = directVel
                    twist.angular.z = velAng

                    self.pub_cmdVel(twist, self.rate_pubVel)

        return 0
    
    def getWaitPoint(self, xRb, yRb, curr_velocity, datalistPointRequestMoveNow):
        numSearch = 400
        longest_distance = 0.
        indexSelect = -1
        lookahead = self.findLookAheadByVel(curr_velocity)
        self.infoGoalNearest, self.disRbToGoalNearest, numpoint = self.findGoalNearest(self.poseRbMa.position.x, self.poseRbMa.position.y, datalistPointRequestMoveNow)
        # print("Hoang ", numpoint, self.infoGoalNearest.indexInListpath)
        if self.startPoint.X == self.finishPoint.X and self.startPoint.Y == self.finishPoint.Y: # check loop
            # find Goal next numSearch step
            numStart = int(self.infoGoalNearest.indexInListpath)
            numEnd = int((self.infoGoalNearest.indexInListpath + numSearch) % numpoint)
            if numEnd <= numStart:
                for i in range(numStart, numpoint, 1):
                    x = datalistPointRequestMoveNow.infoPoint[i].pose.position.x
                    y = datalistPointRequestMoveNow.infoPoint[i].pose.position.y
                    dis = self.calculate_distance(x, y, xRb, yRb)
                    if dis <= lookahead and dis >= longest_distance:
                        longest_distance = dis
                        indexSelect = i
                
                for i in range(0, numEnd, 1):
                    x = datalistPointRequestMoveNow.infoPoint[i].pose.position.x
                    y = datalistPointRequestMoveNow.infoPoint[i].pose.position.y
                    dis = self.calculate_distance(x, y, xRb, yRb)
                    if dis <= lookahead and dis >= longest_distance:
                        longest_distance = dis
                        indexSelect = i

            else:
                for i in range(numStart, numEnd, 1):
                    x = datalistPointRequestMoveNow.infoPoint[i].pose.position.x
                    y = datalistPointRequestMoveNow.infoPoint[i].pose.position.y
                    dis = self.calculate_distance(x, y, xRb, yRb)
                    if dis <= lookahead and dis >= longest_distance:
                        longest_distance = dis
                        indexSelect = i

            if indexSelect == -1:
                for i in range(0, numpoint, 1):
                    x = datalistPointRequestMoveNow.infoPoint[i].pose.position.x
                    y = datalistPointRequestMoveNow.infoPoint[i].pose.position.y
                    dis = self.calculate_distance(x, y, xRb, yRb)
                    if dis >= lookahead:
                        longest_distance = dis
                        indexSelect = i
                        break

        else:
            # find goal to goal finish
            numStart = int(self.infoGoalNearest.indexInListpath)
            # print("Hoang ", numStart, numpoint)
            for i in range(numStart,  numpoint, 1):
                x = datalistPointRequestMoveNow.infoPoint[i].pose.position.x
                y = datalistPointRequestMoveNow.infoPoint[i].pose.position.y
                id = datalistPointRequestMoveNow.infoPoint[i].pathID
                # dis = self.calculate_distance(self.infoGoalNearest.X, self.infoGoalNearest.Y, xRb, yRb)
                dis = self.calculate_distance(self.infoGoalNearest.X, self.infoGoalNearest.Y, x, y)
                if self.nextPointIsStopGoal and id == self.infoGoalStop.path2ID:
                    indexSelect = i - 1
                    break

                if dis >= lookahead or (i == numpoint - 1 and dis <= lookahead):
                    longest_distance = dis
                    indexSelect = i
                    break

        # print(self.startPoint.X , self.startPoint.Y,  self.finishPoint.X, self.finishPoint.Y, lookahead, curr_velocity)
        if indexSelect == -1:
            return False
        
        else:
            self.infoPathFollow.indexInListpath = indexSelect
            self.infoPathFollow.X = datalistPointRequestMoveNow.infoPoint[indexSelect].pose.position.x
            self.infoPathFollow.Y = datalistPointRequestMoveNow.infoPoint[indexSelect].pose.position.y

            self.infoPathFollow.movableZone = datalistPointRequestMoveNow.infoPoint[indexSelect].movableZone
            self.infoPathFollow.fieldSafety = datalistPointRequestMoveNow.infoPoint[indexSelect].fieldSafety
            self.infoPathFollow.direction = datalistPointRequestMoveNow.infoPoint[indexSelect].direction

            pathID = datalistPointRequestMoveNow.infoPoint[indexSelect].pathID
            typePath = datalistPointRequestMoveNow.infoPoint[indexSelect].typePath
            if pathID != self.infoPathFollow.pathID and typePath == 1:
                self.flagCheckAngle = True

            self.infoPathFollow.pathID = pathID
            self.infoPathFollow.typePath = typePath
            # print(self.infoPathFollow.typePath)
            # kiem tra id goal tiep theo co phai Stop khong
            if self.nextPointIsStopGoal == False and self.findStopGoal(self.infoPathFollow.pathID, datalistPointRequestMoveNow):
                self.nextPointIsStopGoal = True

            if self.flagCheckAngle:
                quat = (datalistPointRequestMoveNow.infoPoint[indexSelect].pose.orientation.x, \
                        datalistPointRequestMoveNow.infoPoint[indexSelect].pose.orientation.y, \
                        datalistPointRequestMoveNow.infoPoint[indexSelect].pose.orientation.z, \
                        datalistPointRequestMoveNow.infoPoint[indexSelect].pose.orientation.w)

                angle = self.angleBetweenRobotAndPath(quat)
                if fabs(angle) <= radians(3.) and self.disRbToGoalNearest <= 0.025:
                    self.flagCheckAngle  = False

            else:         
                if self.statusVel != 2:
                    # set vel by %
                    # vel = (datalistPointRequestMoveNow.infoPoint[indexSelect].velocity/100.)*self.max_linearVelocity
                    vel = datalistPointRequestMoveNow.infoPoint[indexSelect].velocity

                    # if vel > 0.25:
                    #     vel = 0.25
                    # vel = 0.2
                    self.infoPathFollow.velocity = self.constrain(vel, self.min_linearVelocity, self.max_linearVelocity)

            return True

    def findGoalAheadType1(self, _a, _b, _c, _x, _y, _dis):
        X_g = Y_g =  X_g1 = Y_g1 = X_g2 = Y_g2 = 0.0
        vectorX = (-1)*_b
        vectorY = _a
        if  _b == 0.0:
            X_g1 = X_g2 = -_c/_a
            Y_g1 = -sqrt(_dis*_dis - (X_g1 - _x)*(X_g1 - _x)) + _y
            Y_g2 = sqrt(_dis*_dis - (X_g2 - _x)*(X_g2 - _x)) + _y
            
        else:
            la = (1.0 + (_a/_b)*(_a/_b))
            lb = -2.0*(_x - (_a/_b)*((_c/_b) + _y))
            lc = _x*_x + ((_c/_b) + _y)*((_c/_b) + _y) - _dis*_dis
            denlta = lb*lb - 4.0*la*lc
            # print(la,lb,lc,denlta)

            X_g1 = (-lb + sqrt(denlta))/(2.0*la)
            X_g2 = (-lb - sqrt(denlta))/(2.0*la)

            Y_g1 = (-_c - _a*X_g1)/_b
            Y_g2 = (-_c - _a*X_g2)/_b

        vectorP1X = _x - X_g1
        vectorP1Y = _y - Y_g1

        if vectorX == 0.0:
            if vectorY*vectorP1Y > 0.0:
                X_g = X_g1
                Y_g = Y_g1
            else:
                X_g = X_g2
                Y_g = Y_g2
        elif vectorY == 0.0:
            if vectorX*vectorP1X > 0.0:
                X_g = X_g1
                Y_g = Y_g1
            else:
                X_g = X_g2
                Y_g = Y_g2
        else:
            v_a = vectorX/vectorP1X
            v_b = vectorY/vectorP1Y
            if v_a*v_b > 0.0 and v_a > 0.0:
                X_g = X_g1
                Y_g = Y_g1
            else:
                X_g = X_g2
                Y_g = Y_g2

        return X_g, Y_g
    
    def moveToTheStopPoint(self, _dis, _poseX, _poseY, _XStop, _YStop, datalistPointRequestMoveNow):
        twist = Twist()
        self.errorAGV = 0
        self.sSafety_AGV = 0
        print(_XStop, _YStop, _dis, self.min_lookahead)
        if self.directFollow == self.directForward and (self.respSafety.zone_ahead == 1 or self.lostSafety):
            self.stop()
            self.sSafety_AGV = 1
            print('not safety')

        else:
            if _dis <= self.min_lookahead:
                velX = self.min_linearVelocity
                self.infoGoalNearest, self.disRbToGoalNearest, numpoint = self.findGoalNearest(self.poseRbMa.position.x, self.poseRbMa.position.y, datalistPointRequestMoveNow)
                infoPointNear = InfoPathFollowing()
                infoPointNear = self.infoGoalNearest

                if self.stepFollowGoalStop == 0:
                    self.saveSTL = StraightLine(Point(infoPointNear.X, infoPointNear.Y), Point(_XStop, _YStop))
                    self.stepFollowGoalStop = 1

                if self.stepFollowGoalStop == 1:
                    xFollow, yFollow = self.saveSTL.findGoalWithLookAhead(_poseX, _poseY, self.min_lookahead)
                    xCVFollow , yCVFollow = self.convert_relative_coordinates(xFollow, yFollow)
                    directVel = velX if self.directFollow == self.directForward else -velX
                    velAng = self.control_navigation(xCVFollow, yCVFollow, directVel)
                    twist.linear.x = directVel
                    twist.angular.z = velAng
                    self.pub_cmdVel(twist, self.rate_pubVel)

            else:
                self.curr_velocity = self.findVelByLookAhead(_dis)
                # self.curr_velocity = self.velSt*(_dis/self.disDeceleration)
                # self.curr_velocity  = self.constrain(self.curr_velocity, self.min_linearVelocity, self.max_linearVelocity)
                print(self.curr_velocity, self.disDeceleration, self.infoPathFollow.velocity, "Follow Target Normal!")
                if self.getWaitPoint(_poseX, _poseY, self.curr_velocity, datalistPointRequestMoveNow):
                    velX = self.curr_velocity
                    xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
                    directVel = velX if self.directFollow == self.directForward else -velX
                    velAng = self.control_navigation(xCVFollow, yCVFollow, directVel)
                    twist.linear.x = directVel
                    twist.angular.z = velAng
                    self.pub_cmdVel(twist, self.rate_pubVel)

                else:
                    self.stop()
                    self.errorAGV = 2
                    print('Error, Ko tim dc waypoint mode follow point stop')
                    return -1
            
        return 1
    
    """
        -1: loi di chuyen 
        0: dung
        1: dang thuc hien
        2. hoan thanh
    """  

    def followPath(self, _ssXNormal, _ssYNormal, _ssXFinish, _ssYFinish, datalistPointRequestMoveNow):
        twist = Twist()
        poseX = self.poseRbMa.position.x
        poseY = self.poseRbMa.position.y
        # angleAGV = self.theta_robotParking
        XStop = 0.
        YStop = 0.
        ss_x = 0.
        ss_y = 0.
        dis = 0.
        disGT = 0.
        mode = ''
        toleranceX = 0.
        toleranceY = 0.
        tolerance_radius = 0.
        self.errorAGV = 0
        self.sSafety_AGV = 0

        if self.nextPointIsStopGoal:
            XStop = self.infoGoalStop.point.position.x
            YStop = self.infoGoalStop.point.position.y
            ss_x = poseX - self.infoGoalStop.point.position.x
            ss_y = poseY - self.infoGoalStop.point.position.y
            toleranceX = _ssXNormal
            toleranceY = _ssYNormal
            mode = 'follow Stop Goal'

        else:
            XStop = self.finishPoint.X
            YStop = self.finishPoint.Y
            ss_x = poseX - self.finishPoint.X
            ss_y = poseY - self.finishPoint.Y
            toleranceX = _ssXFinish
            toleranceY = _ssYFinish
            mode = 'follow Finish Goal'

        dis = sqrt(ss_x*ss_x + ss_y*ss_y)
        tolerance_radius = sqrt(toleranceX*toleranceX + toleranceY*toleranceY)
        disGT = self.curr_velocity*self.decelerationRatio
        maxDisGT = self.curr_velocity*(self.max_lookahead/self.max_linearVelocity)
        if disGT <= maxDisGT:
            disGT = maxDisGT

        if self.flagFollowPointStop == False and dis <= disGT:
            self.velSt = self.curr_velocity
            self.disDeceleration = disGT
            self.flagFollowPointStop = True

        # xet them dk den target - tranh truong hop di qua
        xcv, ycv = self.convert_relative_coordinates(XStop, YStop)
        isFinish = False
        if self.flagFollowPointStop:
            if self.directFollow == self.directForward:
                if xcv <= toleranceX and fabs(ycv) < 0.15:
                    isFinish = True
            elif self.directFollow == self.directReverse: 
                if xcv >= -toleranceX and fabs(ycv) < 0.15:
                    isFinish = True

        print(ss_x, ss_y, dis, self.disDeceleration, mode)
        if (fabs(ss_x) <= toleranceX and fabs(ss_y) <= toleranceY) or dis <= tolerance_radius or isFinish:
            self.stop()
            self.stepFollowGoalStop = 0. # chung
            self.flagFollowPointStop = False # chung
            if self.nextPointIsStopGoal:
                self.nextPointIsStopGoal = False
                print("Done Achieve Stop Goal! next Step is continue to follow the next path (^-^)")
                return 2 # den diem Stop Done
            
            else:
                print(self.finishPoint.X, self.finishPoint.Y, self.target_x, self.target_y)
                if self.finishPoint.X == self.target_x and self.finishPoint.Y == self.target_y: # hoan thanh diem target
                    print("Done Achieve Target Goal! next Step is rotate target angle (^-^)")
                    return 5
                
                else:
                    print("Done Achieve Finish Goal! next Step is waiting request Traffic(^-^)")
                    return 3 # het duong dan, cho duong dan tiep theo
        
        else:
            if self.flagFollowPointStop:
                self.statusVel = 0
                stt = self.moveToTheStopPoint(dis, poseX, poseY, XStop, YStop, datalistPointRequestMoveNow)
                if stt == -1:
                    self.stop()
                    print("Something went wrong (T-T)")
                    return -2

            else:
                # kiem tra dang follow point cuoi chua
                # self.checkFollowPointFinish()
                # them phuong trinh giam 
                velCmd = 0.
                if self.directFollow == self.directForward:
                    if self.respSafety.zone_ahead == 1 or self.lostSafety:
                        self.isDecelerationObstacles = 1
                        velCmd = 0.
                        self.statusVel = 0
                        print('not safety')
                    
                    elif self.respSafety.zone_ahead == 2:
                        if self.isDecelerationObstacles == 1:
                            self.saveVelWhenDecObs = self.infoPathFollow.velocity*(1./3.)
                            self.statusVel = 0

                        elif self.isDecelerationObstacles != 2:
                            self.saveVelWhenDecObs = self.curr_velocity*(1./3.)
                            self.statusVel = 0

                        velCmd = self.saveVelWhenDecObs
                        if velCmd <= self.min_linearVelocity:
                            velCmd = self.min_linearVelocity

                        self.isDecelerationObstacles = 2
                        print('in safety zone 2!')
                    
                    elif self.respSafety.zone_ahead == 3:
                        if self.isDecelerationObstacles == 1 or self.isDecelerationObstacles == 2:
                            self.saveVelWhenDecObs = self.infoPathFollow.velocity*(2./3.)
                            self.statusVel = 0
                        
                        elif self.isDecelerationObstacles != 3:
                            self.saveVelWhenDecObs = self.curr_velocity*(2./3.)
                            self.statusVel = 0

                        velCmd = self.saveVelWhenDecObs
                        if velCmd <= self.min_linearVelocity:
                            velCmd = self.min_linearVelocity

                        self.isDecelerationObstacles = 3
                        print('in safety zone 3!')

                    else:
                        if self.isDecelerationObstacles != 0:
                            self.isDecelerationObstacles = 0
                            self.statusVel = 0

                        velCmd = self.infoPathFollow.velocity

                if self.curr_velocity < velCmd and self.statusVel != 1:
                    self.statusVel = 1
                    self.velSt = self.curr_velocity
                    self.saveTimeVel = rospy.get_time()

                elif self.curr_velocity > velCmd and self.statusVel != 2:
                    self.statusVel = 2
                    self.velSt = self.curr_velocity
                    self.saveTimeVel = rospy.get_time()

                elif self.curr_velocity == velCmd:
                    self.statusVel = 0

                if self.isDecelerationObstacles == 1:
                    self.curr_velocity = 0
                else:
                    self.curr_velocity = self.getVeloctity(velCmd, self.statusVel)
                print(self.curr_velocity, self.statusVel, self.infoPathFollow.velocity, velCmd)

                if self.curr_velocity != 0. :
                    if self.getWaitPoint(poseX, poseY, self.curr_velocity, datalistPointRequestMoveNow):
                        velX = self.curr_velocity
                        xCVFollow , yCVFollow = self.convert_relative_coordinates(self.infoPathFollow.X, self.infoPathFollow.Y)
                        directVel = velX if self.directFollow == self.directForward else -velX
                        velAng = self.control_navigation(xCVFollow, yCVFollow, directVel)
                        twist.linear.x = directVel
                        twist.angular.z = velAng

                        self.pub_cmdVel(twist, self.rate_pubVel)
                    
                    else:
                        self.stop()
                        self.errorAGV = 2
                        print("Something went wrong (T-T)")
                        return -1
                else:
                    self.sSafety_AGV = 1
                    self.stop()
                    
        return 1
    
    # di chuyen toi Point
    def moveToPoint(self, _x, _y, _ssX, _ssY, _vel):
        poseX = self.poseRbMa.position.x
        poseY = self.poseRbMa.position.y
        ss_x = poseX - _x
        ss_y = poseY - _y
        dis = sqrt(ss_x*ss_x + ss_y*ss_y)
        self.errorAGV = 0
        self.sSafety_AGV = 0
        # them dk dung 
        xcv, ycv = self.convert_relative_coordinates(_x, _y)
        isFinish = False
        if xcv <= _ssX and fabs(ycv) < 0.15:
            isFinish = True
        # kiem tra vi tri:
        if (fabs(ss_x) <= _ssX and fabs(ss_y) <= _ssY) or dis <= sqrt(_ssX*_ssX + _ssY*_ssY) or isFinish:
            self.stepMoveToPoint = 0
            self.stop()
            print("Done Move To Point!")
            return 1
        
        else:
            # quay toi Point
            if self.stepMoveToPoint == 0: #tinh goc can toi
                self.saveAngle = self.angleStraightLine(poseX, poseY, _x, _y)
                self.stepMoveToPoint = 1
            
            elif self.stepMoveToPoint == 1: # quay goc
                angle = self.angleBetweenRobotAndPath2(self.saveAngle)
                if self.rotary_around(angle, 0.15, radians(0.5)): # kiem tra goc AGV voi quy dao
                    self.stepMoveToPoint = 2

            elif self.stepMoveToPoint == 2: # tim quy dao di chuyen
                self.saveSTL = StraightLine(Point(poseX, poseY), Point(_x, _y))
                self.saveTimeVel = rospy.get_time()
                self.stepMoveToPoint = 3
            
            elif self.stepMoveToPoint == 3:
                twist = Twist()
                self.curr_velocity = _vel
                lookAhead = self.findLookAheadByVel(self.curr_velocity)
                if self.respSafety.zone_ahead == 1 or self.lostSafety :
                    self.sSafety_AGV = 1
                    self.stop()
                    print('not safety')

                else:
                    xFollow, yFollow = self.saveSTL.findGoalWithLookAhead(poseX, poseY, lookAhead)
                    xCVFollow , yCVFollow = self.convert_relative_coordinates(xFollow, yFollow)
                    velX = self.curr_velocity
                    velAng = self.control_navigation(xCVFollow, yCVFollow, velX)
                    twist.linear.x = velX
                    twist.angular.z = velAng
                    self.pub_cmdVel(twist, self.rate_pubVel)

        return 0
    
    def findStopGoal(self, idFollow, datalistPointRequestMoveNow):
        num = len(datalistPointRequestMoveNow.infoStopPoint)
        if num:
            for i in range(num):
                pathID = datalistPointRequestMoveNow.infoStopPoint[i].pathID
                if idFollow == pathID:
                    self.infoGoalStop = datalistPointRequestMoveNow.infoStopPoint[i]
                    return True
                
        return False  
    

    def checkPathOutOfRange(self, datalistPointRequestMoveNow):
        self.errorAGV = 0
        self.infoGoalNearest, self.disRbToGoalNearest, numpoint = self.findGoalNearest(self.poseRbMa.position.x, self.poseRbMa.position.y, datalistPointRequestMoveNow)
        goalNearNow = InfoPathFollowing()
        goalNearNow = self.infoGoalNearest
        disNear = self.disRbToGoalNearest
        print(goalNearNow.X, self.finishPoint.X, goalNearNow.Y, self.finishPoint.Y, goalNearNow.pathID, self.finishPoint.pathID)

        if disNear >= goalNearNow.movableZone:
            self.errorAGV = 1
            return -1 # bao loi qua khoang cachs -> yeu cau di chuyen gan lai

        if goalNearNow.X == self.finishPoint.X and goalNearNow.Y == self.finishPoint.Y: # la diem cuoi
            self.errorAGV = 10
            return -2 # bao loi AGV qua diem goal
        
        if goalNearNow.pathID == self.finishPoint.pathID: # la diem cuoi
            a = 0.84
            b = 1.491
            dis = self.calculate_distance(goalNearNow.X, goalNearNow.Y, self.finishPoint.X, self.finishPoint.Y)
            if dis < a*disNear + b:
                return 2
            
        return 1

    def resetAll(self):
        self.completed_all = False
        self.completed_simple = 0
        self.completed_backward = 0
        self.completed_reset = False
        self.errorAGV = 0
        self.sSafety_AGV = 0
        self.nextPointIsStopGoal = False
        self.flagVelChange = True
        self.flagFollowPointStop = False
        self.flagCheckAngle = False
        self.stepMoveToPoint = 0
        self.stepCheckStart = 0
        self.stepFollowGoalStop = 0
        self.stepMoveForward = 0
        self.target_x = self.coordinate_unknown
        self.target_y = self.coordinate_unknown
        self.end_of_list = False
        self.isDecelerationObstacles = 0

        # -- 
        self.statusVel = 0
        self.curr_velocity = 0.
        self.velSt = 0.
        self.saveTimeVel = rospy.get_time()

    def updateAll(self):
        self.completed_all = False
        self.completed_simple = 0
        self.completed_backward = 0
        self.completed_reset = False
        self.errorAGV = 0
        self.sSafety_AGV = 0
        self.flagVelChange = True
        self.flagFollowPointStop = False
        self.flagCheckAngle = False
        self.nextPointIsStopGoal = False
        self.stepMoveToPoint = 0
        self.stepCheckStart = 0
        self.stepFollowGoalStop = 0
        self.stepMoveForward = 0
        self.end_of_list = False
        self.isDecelerationObstacles = 0

        # -- 
        self.statusVel = 0
        self.curr_velocity = 0.
        self.velSt = 0.
        self.saveTimeVel = rospy.get_time()

    def checkResetOrTargetChange(self, datalistPointRequestMoveNow):
        targetX = round(datalistPointRequestMoveNow.target_x, 3)
        targetY = round(datalistPointRequestMoveNow.target_y, 3)
        targetZ = round(datalistPointRequestMoveNow.target_z, 3)
        if datalistPointRequestMoveNow.enable == 0 or self.target_x != targetX or self.target_y != targetY:
            self.stop()
            return 1
        
        return 0
    
    def run(self):
        self.process = 1
        # while not rospy.is_shutdown():    
        while not self.shutdown_flag.is_set(): 
            if self.process == 0:
                ck = 0
                if self.is_pose_robot:
                    ck += 1
                if self.is_safetyZone:
                    ck += 1
                if ck == 2:
                    self.process = 1
                    rospy.loginfo("Done receive all need data <(^-^)> ")

            elif self.process == 1:
                if self.is_listPointRequestMove:
                    self.datalistPointRequestMoveNow = copy.copy(self.datalistPointRequestMove)
                    # if self.enb == 0:
                        # print("co tin hieu reset",  self.datalistPointRequestMoveNow.enable, self.enb)
                        
                    if self.datalistPointRequestMoveNow.enable == 0: # tin hieu reset
                        if not self.completed_reset:
                            self.stop()
                            self.resetAll()
                            self.completed_reset = True
                            print("Done Reset!")

                    elif self.datalistPointRequestMoveNow.enable == 1: # di chuyen theo line
                        self.process = 19

                    elif self.datalistPointRequestMoveNow.enable == 2: # di chuyen ra khoi pallet
                        self.process = 22

                    else:
                        self.stop()
                        self.resetAll()

            ## them buoc cap nhat target and list Move
            elif self.process == 19:
                self.datalistPointRequestMoveNow = copy.copy(self.datalistPointRequestMove)
                targetX = round(self.datalistPointRequestMoveNow.target_x, 3)
                targetY = round(self.datalistPointRequestMoveNow.target_y, 3)
                targetZ = round(self.datalistPointRequestMoveNow.target_z, 3)
                if self.datalistPointRequestMoveNow.enable == 1:
                    print("Before!", self.target_x, self.target_y, targetX, targetY)
                    if targetX < 500.0 and targetY < 500.0:
                        if self.target_x != targetX or self.target_y != targetY:
                            self.stop()
                            self.updateAll()
                            self.target_x = targetX
                            self.target_y = targetY
                            self.target_z = targetZ
                            self.saveStartPoint = copy.copy(self.startPoint)
                            self.saveFinishPoint = copy.copy(self.finishPoint)
                            print("Recive new Target and Plan!", self.target_x, self.target_y)

                        self.process = 20

                    else:
                        self.process = 1

                else:
                    self.process = 1

            # Buoc kiem tra xu ly process 
            elif self.process == 20:
                if self.completed_all:
                    self.process = 1

                else:
                    if (self.startPoint.X != self.saveStartPoint.X or self.startPoint.Y != self.saveStartPoint.Y or \
                        self.finishPoint.X != self.saveFinishPoint.X or self.finishPoint.Y != self.saveFinishPoint.Y):
                        if (self.finishPoint.X != self.saveFinishPoint.X and self.finishPoint.Y != self.saveFinishPoint.Y):
                            self.end_of_list = False

                        self.saveStartPoint = copy.copy(self.startPoint)
                        self.saveFinishPoint = copy.copy(self.finishPoint)
                        print("Update new list!")

                    if (self.end_of_list):
                        self.process = 1

                    else:
                        self.process = 12
    
            elif self.process == 12:
                self.datalistPointRequestMoveNow = copy.copy(self.datalistPointRequestMove)
                stt = self.checkPathOutOfRange(self.datalistPointRequestMoveNow)
                if stt == -1:
                    self.process = 1
                    print("OUT OF PATH")
                if stt == 1: # di chuyen bam duong dan
                    self.process = 15
                    print("di chuyen bam duong", self.process)
                elif stt == 2: # di chuyen huong den diem finish
                    self.process = 13
                    print("di chuyen huong den diem finish", self.process)
                elif stt == -2:
                    print("agv di qua diem goal")

            elif self.process == 13: # di chuyen den vi tri finish
                self.datalistPointRequestMoveNow = copy.copy(self.datalistPointRequestMove)
                isChangeOrReset = self.checkResetOrTargetChange(self.datalistPointRequestMoveNow)
                if isChangeOrReset == 0:
                    stt = self.moveToPoint(self.finishPoint.X, self.finishPoint.Y, self.toleranceByX_PointFinish, self.toleranceByY_PointFinish, self.min_linearVelocity)
                    if stt == 1: # done
                        if self.finishPoint.X == self.target_x and self.finishPoint.Y == self.target_y:
                            self.process = 17 # quay dap ung goc cuoi
                        else:
                            self.process = 18 # di chuyen het duong dan.. wait cap nhat
                            print("waiting update!")
                    elif stt == -1: # error
                        print("error", self.process)
                else:
                    self.process = 1

            elif self.process == 101: # cho doi lenh tu server
                if self.datalistPointRequestMove.enableStop == 0:
                    print("enableStop = 0 so Start Moving!")
                    self.process = 1

            elif self.process == 15: # kiem tra vi tri AGV voi duong dan 
                self.datalistPointRequestMoveNow = copy.copy(self.datalistPointRequestMove)
                isChangeOrReset = self.checkResetOrTargetChange(self.datalistPointRequestMoveNow)
                if isChangeOrReset== 0:
                    stt = self.moveIntoPath(self.datalistPointRequestMoveNow)
                    if stt == 1:
                        print("Done move into path")
                        print(self.curr_velocity, self.statusVel)
                        self.process = 16
                else:
                    self.process = 1

            elif self.process == 16: # follow theo duong dan
                self.datalistPointRequestMoveNow = copy.copy(self.datalistPointRequestMove)
                isChangeOrReset = self.checkResetOrTargetChange(self.datalistPointRequestMoveNow)
                if isChangeOrReset== 0:
                    # kiem tra xem co diem dung tranh khong
                    if self.datalistPointRequestMoveNow.enableStop == 1:
                        print("dung tai day")
                        self.statusVel = 0
                        self.pointStopAvoid.x = self.datalistPointRequestMoveNow.pointStopAvoid.point.position.x
                        self.pointStopAvoid.y = self.datalistPointRequestMoveNow.pointStopAvoid.point.position.y
                        self.stop()

                    else:
                        stt = self.followPath(self.toleranceByX_PointStop, self.toleranceByY_PointStop, self.toleranceByX_PointFinish, self.toleranceByY_PointFinish, self.datalistPointRequestMoveNow)
                        if stt == 2: # den diem stop point
                            self.process = 15
                        elif stt == 3: # den diem cuoi trong list
                            self.process = 18 # di chuyen het duong dan.. wait cap nhat
                        elif stt == 5: # den diem target
                            self.process = 17 # quay dap ung goc cuoi
                        elif stt == -1: # loi
                            self.process = -2
                        elif stt == -2: # loi
                            self.process = -3

                else:
                    self.process = 1

            elif self.process == 18:
                self.end_of_list = True
                self.process = 1

            elif self.process == 17:
                # self.datalistPointRequestMoveNow = copy.copy(self.datalistPointRequestMove)
                # isChangeOrReset = self.checkResetOrTargetChange(self.datalistPointRequestMoveNow)
                # if isChangeOrReset== 0:
                #     theta = self.target_z - self.theta_robotNow
                #     if fabs(theta) >= PI:
                #         theta_t = (2*PI - fabs(theta))
                #         if theta > 0:
                #             theta = -theta_t
                #         else:
                #             theta = theta_t
                    
                    # if self.rotary_around(theta, 0.15, radians(1.)): # kiem tra goc AGV voi quy dao
                        self.completed_all = True
                        self.process = 1
                #         print("DONE ROTATY, WAIT NEW TARGET! (^-^)")

                # else:
                #     self.process = 1

            elif self.process == 22:
                if self.completed_backward == 0:
                    self.completed_reset = False
                    if self.statusAGV == 0:
                        if self.datalistPointRequestMove.target_x < 500 and self.datalistPointRequestMove.target_y < 500:
                            self.target_x = self.datalistPointRequestMove.target_x
                            self.target_y = self.datalistPointRequestMove.target_y
                            self.process = 23
                            self.statusAGV = 1
                        
                        else:
                            self.stop()
                            self.process = 1

                    elif self.statusAGV == 1:
                        self.process = 23

                else:
                    self.statusAGV = 0
                    self.process = 1
            
            elif self.process == 23:
                stt = 2
                # stt = self.straightOutOfPallet(self.target_x, self.target_y, 1800)
                if stt == 2:
                    self.statusAGV = 0
                    self.completed_backward = 1

                self.process = 1

            elif self.process == -2:
                print("Error, Ko tim dc waypoint mode navi")
                if self.datalistPointRequestMoveNow.enable == 0:
                    self.process = 1

            elif self.process == -3:
                print("Error, Ko tim dc waypoint mode follow point stop")
                if self.datalistPointRequestMoveNow.enable == 0:
                    self.process = 1

            # self.pubMakerPointFollow(self.infoPathFollow.X, self.infoPathFollow.Y)
            if self.datalistPointRequestMoveNow.enable == 0:
                self.fieldRequest.data = 0
                self.pub_status(self.target_x, self.target_y, self.target_z, self.datalistPointRequestMoveNow.enable,0,0,0,0,0,0., 0)

            elif self.datalistPointRequestMoveNow.enable == 1 or self.datalistPointRequestMoveNow.enable == 3:
                self.fieldRequest.data = self.infoGoalNearest.fieldSafety
                self.pub_status(self.target_x, self.target_y, self.target_z, self.datalistPointRequestMoveNow.enable,self.process,self.errorAGV,self.sSafety_AGV,self.completed_all,self.infoPathFollow.pathID, self.path_error, self.infoPathFollow.typePath)   

            elif self.datalistPointRequestMoveNow.enable == 2:
                self.fieldRequest.data = self.infoGoalNearest.fieldSafety
                self.pub_status(self.target_x, self.target_y, self.target_z, self.datalistPointRequestMoveNow.enable,self.process,self.errorAGV,self.sSafety_AGV,self.completed_backward,0, 0., 0)   

            self.pubFieldSafety.publish(self.fieldRequest)

            # check status theard
            self.saveTimeMainTheard = rospy.get_time()
            self.startLoop = False
            self.rate.sleep()

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

    rospy.init_node('goal_control', anonymous=False)
    print("initial node!")

    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:

        br = GoalControl(1)
        br.start()

        # Keep the main thread running, otherwise signals are ignored.
        while not rospy.is_shutdown():
            # find goal nearest
            # if br.is_pose_robot and br.is_listPointRequestMove:
            #     br
            #     if br.datalistPointRequestMove.enable == 1:
            #         br.infoGoalNearest, br.disRbToGoalNearest, br.isGoalNearest, numpoint = br.findGoalNearest(br.poseRbMa.position.x, br.poseRbMa.position.y)
            #         br.isGoalNearest = True
            #     else:
            #         br.isGoalNearest = False
            # check theard
            if br.checkLostMainTheard():
                br.stop()
                print('Theard dead')

            # check safety
            if br.checkLostSafety():
                print('Lost data safety')
                br.stop()

            time.sleep(0.05)
 
    except ServiceExit:
        
        br.shutdown_flag.set()
        # Wait for the threads to close...
        br.join()
    print('Exiting main program')

if __name__ == '__main__':
    main()


