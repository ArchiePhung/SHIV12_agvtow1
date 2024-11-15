#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from sti_msgs.msg import PathInfo, PointRequestMove, ListPointRequestMove, LineRequestMove, StopPoint
# from message_pkg.msg import Parking_request, Parking_respond
import numpy as np
from math import sqrt, pow, atan, acos, modf, atan2, fabs, tan, degrees, radians
from math import pi as PI
# import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import cmath

class Point():
  def __init__(self, _x=0, _y=0):
    self.x = _x
    self.y = _y

class PoseXYZ():
  def __init__(self, _x=0, _y=0, _z=0):
    self.x = _x
    self.y = _y
    self.z = _z

class Paraboll():
    def __init__(self, _pointOne=Point(), _pointSecond=Point()):
        #khoi tao 2 bien point
        self.pointOne = _pointOne
        self.pointSecond = _pointSecond

        b = np.array([self.pointOne.y, self.pointSecond.y, 0])
        b = b[:, np.newaxis]
        A = np.array([[pow(self.pointOne.x, 2), self.pointOne.x, 1], [pow(self.pointSecond.x, 2), self.pointSecond.x, 1], [2*self.pointSecond.x, 1, 0]])
        C = np.dot(np.linalg.inv(A), b)
        print(C)
        self.a = C[0,0]
        self.b = C[1,0]
        self.c = C[2,0]

    def calc(self, _x):
        y = self.a*pow(_x, 2) + self.b*_x + self.c
        return y
    
    def tangent(self, _point):
        a = 2*self.a*_point.x + self.b
        if _point.y > 0:
            return -atan(a)
        else:
            return atan(a)
        
    def slip(self, _step):
        listPoint = np.empty((0,2), dtype=float)
        rx = np.arange(self.pointOne.x, self.pointSecond.x, -_step, float)
        for i in rx:
            listPoint = np.append(listPoint, np.array([[i, self.calc(i)]]), axis=0)
        return listPoint
        
class StraightLine():
    def __init__(self, _pointOne=Point(), _pointSecond=Point()):
        self.pointOne = _pointOne
        self.pointSecond = _pointSecond

        self.a = self.pointOne.y - self.pointSecond.y
        self.b = self.pointSecond.x - self.pointOne.x
        self.c = -self.pointOne.x*self.a - self.pointOne.y*self.b

        print("Make a Straight Line")
        # print(self.a,self.b,self.c)

        self.dis = sqrt(self.a*self.a + self.b*self.b)

    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)
    
    def calc(self, _x):
        pass

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

    def slip(self, _l):
        X_g = Y_g =  X_g1 = Y_g1 = X_g2 = Y_g2 = 0.0
        decimal, numpart = modf(self.dis/float(_l))
        listPoint = np.array([[self.pointOne.x, self.pointOne.y]])
        for i in range(int(numpart)):
            l = (i+1)*_l
            if self.b == 0.:
                X_g1 = X_g2 = -self.c/self.a
                Y_g1 = -sqrt(l*l - (X_g1 - self.pointOne.x)*(X_g1 - self.pointOne.x)) + self.pointOne.y
                Y_g2 = sqrt(l*l - (X_g2 - self.pointOne.x)*(X_g2 - self.pointOne.x)) + self.pointOne.y

            else:
                la = (1.0 + (self.a/self.b)*(self.a/self.b))
                lb = -2.0*(self.pointOne.x - (self.a/self.b)*((self.c/self.b) + self.pointOne.y))
                lc = self.pointOne.x*self.pointOne.x + ((self.c/self.b) + self.pointOne.y)*((self.c/self.b) + self.pointOne.y) - l*l
                denlta = lb*lb - 4.0*la*lc
                # print(la,lb,lc,denlta)

                X_g1 = (-lb + sqrt(denlta))/(2.0*la)
                X_g2 = (-lb - sqrt(denlta))/(2.0*la)

                Y_g1 = (-self.c - self.a*X_g1)/self.b
                Y_g2 = (-self.c - self.a*X_g2)/self.b

            if self.checkPointInLine(X_g1, Y_g1):
                X_g = X_g1
                Y_g = Y_g1
            else:
                X_g = X_g2
                Y_g = Y_g2

            listPoint = np.append(listPoint, np.array([[X_g, Y_g]]), axis=0)
            # np.append(listPointX, X_g)
            # np.append(listPointY, Y_g)

        if decimal != 0.:
            listPoint = np.append(listPoint, np.array([[self.pointSecond.x, self.pointSecond.y]]), axis=0)

        return listPoint

    def checkPointInLine(self, _pointX, _pointY):
        if _pointX == self.pointSecond.x and _pointY == self.pointSecond.y:
            return True
        
        if _pointX == self.pointOne.x and _pointY == self.pointOne.y:
            return True
        
        # loai nghiem bang vector
        vector_qd_x = self.pointOne.x - self.pointSecond.x
        vector_qd_y = self.pointOne.y - self.pointSecond.y

        vector_point1_x = self.pointOne.x - _pointX
        vector_point1_y = self.pointOne.y - _pointY

        if vector_qd_x == 0.0:
            if vector_qd_y*vector_point1_y > 0.0:
                return True
            
        elif vector_qd_y == 0.0:
            if vector_qd_x*vector_point1_x > 0.0:
                return True

        else:
            v_a = vector_qd_x/vector_point1_x
            v_b = vector_qd_y/vector_point1_y
            if v_a*v_b > 0.0 and v_a > 0.0:
                return True
        
        # angle = self.calAngleThreePoint(self.pointOne.x, self.pointOne.y, _pointX, _pointY, self.pointSecond.x, self.pointSecond.y)
        # if angle >= 165.*PI/180.:
        #     return True

        return False
    

class Circle():
    def __init__(self, _pointCenter=Point(), _radius=0.):

        self.pointCenter = _pointCenter
        self.radius = _radius

        self.a = -2*self.pointCenter.x
        self.b = -2*self.pointCenter.y
        self.c = self.pointCenter.x*self.pointCenter.x + self.pointCenter.y*self.pointCenter.y - self.radius*self.radius

        print("Make a Circle")

    def calAngleThreePoint(self, x1, y1, x2, y2, x3, y3):
        dx1 = x1 - x2
        dy1 = y1 - y2
        dx2 = x3 - x2
        dy2 = y3 - y2
        c_goc = (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-12)
        goc = acos(c_goc)
        
        return goc
    
    def calc(self, count):
        for i in range(count):
            r = cmath.rect(self.radius, (2*cmath.pi)*(i/count))
            print([round(self.pointCenter.x+r.real, 2), round(self.pointCenter.y+r.imag, 2)])

    def convertAngle(self, _angle):
        if _angle > PI:
            return _angle - 2.*PI
        return _angle
    
    def slip(self, _pointOne, _pointSecond, _l):
        xStart_cv = _pointOne.x - self.pointCenter.x
        yStart_cv = _pointOne.y - self.pointCenter.y

        xFinish_cv = _pointSecond.x - self.pointCenter.x
        yFinish_cv = _pointSecond.y - self.pointCenter.y

        degree = _l/self.radius
        angleStartIn2PI = self.findAngleOfCircle(xStart_cv, yStart_cv)
        angleFinishIn2PI = self.findAngleOfCircle(xFinish_cv, yFinish_cv)
        dentalA = angleFinishIn2PI - angleStartIn2PI
        if fabs(dentalA) > PI:
            dentalA = 2.*PI - fabs(dentalA)
        decimal, step = modf(fabs(dentalA)/degree)
        angleStart = self.convertAngle(angleStartIn2PI)
        angleFinish = self.convertAngle(angleFinishIn2PI)
        print(angleStart, angleFinish)
        dtAngle = angleFinish - angleStart
        if fabs(dtAngle) > PI:
            if dtAngle > 0:
                direct = -1
            else:
                direct = 1
        else:
            if dtAngle > 0:
                direct = 1
            else:
                direct = -1

        # dtAngle = angleFinish - angleStart
        # if fabs(dtAngle) > PI:
        #     if angleStart > angleFinish:
        #         direct = 1
        #         for i in np.arange(angleStart, angleFinish, direct*degree):
        #             r = cmath.rect(self.radius, i)
        #             listPoint = np.append(listPoint, np.array([[round(self.pointCenter.x+r.real, 3), round(self.pointCenter.y+r.imag, 3)]]), axis=0)
        #     else:
        #         direct = -1

        # else:

        #     if angleStart > angleFinish:
        #         direct = -1
        #         for i in np.arange(angleStart, angleFinish, direct*degree):
        #             r = cmath.rect(self.radius, i)
        #             listPoint = np.append(listPoint, np.array([[round(self.pointCenter.x+r.real, 3), round(self.pointCenter.y+r.imag, 3)]]), axis=0)
        #     else:
        #         direct = 1
        #         for i in np.arange(angleStart, angleFinish, direct*degree):
        #             r = cmath.rect(self.radius, i)
        #             listPoint = np.append(listPoint, np.array([[round(self.pointCenter.x+r.real, 3), round(self.pointCenter.y+r.imag, 3)]]), axis=0)

        # print(direct)
        listPoint = np.empty((0,2), dtype=float)
        # for i in np.arange(angleStart, angleFinish, direct*degree):
        #     r = cmath.rect(self.radius, i)
        #     listPoint = np.append(listPoint, np.array([[round(self.pointCenter.x+r.real, 3), round(self.pointCenter.y+r.imag, 3)]]), axis=0)

        for i in range(int(step)+1):
            angle = angleStart + direct*i*degree
            r = cmath.rect(self.radius, angle)
            listPoint = np.append(listPoint, np.array([[round(self.pointCenter.x+r.real, 3), round(self.pointCenter.y+r.imag, 3)]]), axis=0)

        return np.append(listPoint, np.array([[_pointSecond.x, _pointSecond.y]]), axis=0)

    def findAngleOfCircle(self, _x, _y):
        if _x == 0.:
            if _y > 0.:
                return PI/2.
            else:
                return 3.*PI/2.
            
        elif _y == 0.:
            if _x > 0.:
                return 0.
            else:
                return PI
            
        else:
            angle = fabs(atan2(_y, _x))
            if _x > 0 and _y > 0:
                return angle
            elif _x > 0 and _y < 0:
                return 2*PI - angle
            elif _x < 0 and _y > 0:
                return PI - angle
            else:
                return PI + angle


class QuadraticBezierCurves():
    def __init__(self, _pointOne=Point(), _pointSecond=Point(), _midpoint=Point(), _typeDefine=0,_angleOne=0., _angleSecond=0., _numberPts=0 ):

        self.pointOne = _pointOne
        self.pointSecond = _pointSecond
        self.midPoint = _midpoint
        if _typeDefine == 1:
            a1, b1, c1 = self.findStraightLineByAngleAndPoint(self.pointOne, _angleOne)
            a2, b2, c2 = self.findStraightLineByAngleAndPoint(self.pointSecond, _angleSecond)
            if b1 == 0:
                x = -c1/a1
                y = (-c2-a2*x)/b2

            else:
                x = ((b2*c1)/b1 - c2)/(a2 - (b2*a1)/b1)
                y = (-c1-a1*x)/b1

            # x = (-c1+b2)/(a1-a2)
            # y = a1*x + c1
            self.midPoint = Point(x,y)
            print(self.midPoint.x ,self.midPoint.y)
        # self.numberPts = _numberPts

        self.numberPts = self.findNumberPts()

        self.t = np.array([i*1/self.numberPts for i in range(0,self.numberPts+1)])

        print("Make a Quadratic Bezier Curves ")

    def findStraightLineByAngleAndPoint(self, _point, _angle):
        if fabs(_angle) == PI/2.:
            return 1, 0, -_point.x
        else:
            k = tan(_angle)
            return k, -1., (-1)*k*_point.x + _point.y


    def findNumberPts(self):
        dis1 = self.calculate_distance(self.pointOne.x, self.pointOne.y, self.midPoint.x, self.midPoint.y)
        dis2 = self.calculate_distance(self.midPoint.x, self.midPoint.y, self.pointSecond.x, self.pointSecond.y)
        dis3 = self.calculate_distance(self.pointOne.x, self.pointOne.y, self.pointSecond.x, self.pointSecond.y)

        return int(max(dis1, dis2, dis3)/0.01)


    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)

    def slip(self):
        listPoint = np.empty((0,2), dtype=float)
        for i in self.t:
            _x = (1-i)*((1-i)*self.pointOne.x + i*self.midPoint.x) + i*((1-i)*self.midPoint.x + i*self.pointSecond.x)
            _y = (1-i)*((1-i)*self.pointOne.y + i*self.midPoint.y) + i*((1-i)*self.midPoint.y + i*self.pointSecond.y)

            listPoint = np.append(listPoint, np.array([[_x, _y]]), axis=0)

        return listPoint


class BSpline():
    pass

class MakePathByPoint():
    def __init__(self):
        rospy.init_node('make_pathByPoint', anonymous=False)
        print("initial node!")
        self.rate = rospy.Rate(20)
        self.topic_subPose = rospy.get_param("~topic_subPose", "robotPose_lidarLOC")
        self.topic_subReq = rospy.get_param("~topic_subReq", "request_move")
        self.topic_pubListP = rospy.get_param("~topic_pubListP", "list_pointRequestMove")
        self.topic_pubPath = rospy.get_param("~topic_pubPath", "path_global")

        # param
        self.angleStop = rospy.get_param('~angleStop',165.) 

        rospy.Subscriber(self.topic_subPose, PoseStamped, self.callback_poseRobot, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.poseStampedAGV = PoseStamped()
        self.theta_robotNow = 0.0

        rospy.Subscriber(self.topic_subReq, LineRequestMove, self.callback_moveRequest, queue_size = 1024)
        data = LineRequestMove() 
        self.is_moveRequest = False

        self.pubPath = rospy.Publisher(self.topic_pubPath, Path, queue_size= 20)
        self.msgPath = Path()
        self.msgPath.header.frame_id = 'map'
        self.msgPath.header.stamp = rospy.Time.now()

        self.pubListPoint = rospy.Publisher(self.topic_pubListP, ListPointRequestMove, queue_size= 20)
        self.msgListPoint = ListPointRequestMove()

        self.pointOne = Point(1., 0.)
        self.pointSecond = Point(5., 0.)

        self.makedParaboll = False

        self.poseRobotX = 0.
        self.poseRobotY = 0.

        self.xFollow = 0.
        self.yFollow = 0.

        self.process = 0

        self.stepPoint = 0.01
        self.straightLineMode = 1
        self.circleMode = 2
        self.quadraticBezierCurvesMode = 3
        self.curverLineMode = 4

        self.makePathDone = False
        self.flagTargetChange = False
        self.flagListPathChange = False

        self.poseTarget = PoseXYZ()
        self.pathInfo = PathInfo()
        self.listIDPath = []

        rospy.spin()

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


    def calAngleThreePoint(self, x1, y1, x2, y2, x3, y3):
        dx1 = x1 - x2
        dy1 = y1 - y2
        dx2 = x3 - x2
        dy2 = y3 - y2
        c_goc = (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-12)
        goc = acos(c_goc)
        
        return goc


    def callback_moveRequest(self, data):
        enb = data.enable
        self.msgListPoint.enable = enb
        self.msgListPoint.target_x = data.target_x
        self.msgListPoint.target_y = data.target_y
        self.msgListPoint.target_z = data.target_z
        self.msgListPoint.offset = data.offset
        self.msgListPoint.enableStop = data.enableStop
        self.msgListPoint.pointStopAvoid = data.pointStopAvoid
        self.msgListPoint.mission = data.mission
        self.msgListPoint.mess = data.mess

        if enb != 0:
            # print("pose request", data.target_x, data.target_y, data.target_z)
            poseTarget = PoseXYZ(data.target_x, data.target_y, data.target_z)
            # print("pose request", data.target_x, data.target_y, data.target_z)
            # print("hehehe", poseTarget.x, poseTarget.y, poseTarget.z)
            if poseTarget.x != self.poseTarget.x or poseTarget.y != self.poseTarget.y or poseTarget.z != self.poseTarget.z:
                self.poseTarget = poseTarget
                self.flagTargetChange = True
                print(self.poseTarget.x, self.poseTarget.y, self.poseTarget.z)

            else:
                self.flagTargetChange = False

            numLen = len(data.pathInfo)
            if numLen != 0:
                listID = []
                for i in range(numLen):
                    listID.append(data.pathInfo[i].pathID)

                if listID != self.listIDPath:
                    self.listIDPath = listID
                    self.pathInfo = data.pathInfo
                    self.flagListPathChange = True

                else:
                    self.flagListPathChange = False

            else:
                self.flagListPathChange = False

            print(self.flagListPathChange, self.flagTargetChange)
        
        else:
            print("reset enable 0 ")
            self.resetAll()

        # print("enennene ", data.enable)

        if self.flagListPathChange or self.flagTargetChange:
            # print("path change ")
            self.resetAll()

        if enb == 1 and self.makePathDone == False:
            num = len(data.pathInfo)
            # xet stop point and point change direct
            if num > 1:
                for i in range(num-1):
                    directL1 = data.pathInfo[i].direction
                    directL2 = data.pathInfo[i+1].direction
                    typeP1 = data.pathInfo[i].typePath
                    typeP2 = data.pathInfo[i+1].typePath

                    if directL1 != directL2:
                        pointStop = StopPoint()
                        pointStop.pathID = data.pathInfo[i].pathID
                        pointStop.path2ID = data.pathInfo[i+1].pathID
                        pointStop.point = data.pathInfo[i].pointSecond
                        print("--------direct Point--------")
                        # line = StraightLine(point2, point3)
                        # print("--------------------------")
                        # orien = line.getQuaternion()
                        # pointStop.point.orientation.z = orien[2]
                        # pointStop.point.orientation.w = orien[3]
                        self.msgListPoint.infoStopPoint.append(pointStop)
                        continue
                    
                    point1 = Point()
                    point2 = Point()
                    point3 = Point()
                    angleValid = 0.
                    isCheckAngle = False
                    if (typeP1 == self.straightLineMode and typeP2 == self.straightLineMode):
                        isCheckAngle = True
                        angleValid = radians(self.angleStop)
                        point1.x = data.pathInfo[i].pointOne.position.x
                        point1.y = data.pathInfo[i].pointOne.position.y
                        point2.x = data.pathInfo[i].pointSecond.position.x
                        point2.y = data.pathInfo[i].pointSecond.position.y
                        point3.x = data.pathInfo[i+1].pointSecond.position.x
                        point3.y = data.pathInfo[i+1].pointSecond.position.y

                    elif (typeP1 == self.straightLineMode and typeP2 == self.quadraticBezierCurvesMode):
                        isCheckAngle = True
                        angleValid = radians(self.angleStop)
                        point1.x = data.pathInfo[i].pointOne.position.x
                        point1.y = data.pathInfo[i].pointOne.position.y
                        point2.x = data.pathInfo[i].pointSecond.position.x
                        point2.y = data.pathInfo[i].pointSecond.position.y
                        point3.x = data.pathInfo[i+1].pointMid.position.x
                        point3.y = data.pathInfo[i+1].pointMid.position.y

                    elif (typeP1 == self.quadraticBezierCurvesMode and typeP2 == self.straightLineMode):
                        isCheckAngle = True
                        angleValid = radians(self.angleStop)
                        point1.x = data.pathInfo[i].pointMid.position.x
                        point1.y = data.pathInfo[i].pointMid.position.y
                        point2.x = data.pathInfo[i].pointSecond.position.x
                        point2.y = data.pathInfo[i].pointSecond.position.y
                        point3.x = data.pathInfo[i+1].pointSecond.position.x
                        point3.y = data.pathInfo[i+1].pointSecond.position.y

                    elif (typeP1 == self.quadraticBezierCurvesMode and typeP2 == self.quadraticBezierCurvesMode):
                        isCheckAngle = True
                        angleValid = radians(self.angleStop)
                        point1.x = data.pathInfo[i].pointMid.position.x
                        point1.y = data.pathInfo[i].pointMid.position.y
                        point2.x = data.pathInfo[i].pointSecond.position.x
                        point2.y = data.pathInfo[i].pointSecond.position.y
                        point3.x = data.pathInfo[i+1].pointMid.position.x
                        point3.y = data.pathInfo[i+1].pointMid.position.y

                    # -- 
                    isCheckAngle = False
                    if isCheckAngle:
                        angle = self.calAngleThreePoint(point1.x, point1.y, point2.x, point2.y, point3.x, point3.y)
                        if angle <= angleValid:
                            pointStop = StopPoint()
                            pointStop.pathID = data.pathInfo[i].pathID
                            pointStop.path2ID = data.pathInfo[i+1].pathID
                            pointStop.point = data.pathInfo[i].pointSecond
                            print("--------stop Point--------")
                            line = StraightLine(point2, point3)
                            print("--------------------------")
                            orien = line.getQuaternion()
                            pointStop.point.orientation.z = orien[2]
                            pointStop.point.orientation.w = orien[3]
                            self.msgListPoint.infoStopPoint.append(pointStop)

                    # typeP1 = data.pathInfo[i].typePath
                    # if typeP1 == self.straightLineMode:
                    #     typeP2 = data.pathInfo[i+1].typePath
                    #     if typeP2 == self.straightLineMode:
                    #         point1 = Point(data.pathInfo[i].pointOne.position.x, data.pathInfo[i].pointOne.position.y)
                    #         point2 = Point(data.pathInfo[i].pointSecond.position.x, data.pathInfo[i].pointSecond.position.y)
                    #         point3 = Point(data.pathInfo[i+1].pointSecond.position.x, data.pathInfo[i+1].pointSecond.position.y)

                    #         angle = self.calAngleThreePoint(point1.x, point1.y, point2.x, point2.y, point3.x, point3.y)
                    #         if angle <= radians(self.angleStop):
                    #             pointStop = StopPoint()
                    #             pointStop.pathID = data.pathInfo[i].pathID
                    #             pointStop.point = data.pathInfo[i].pointSecond
                    #             print("--------stop Point--------")
                    #             line = StraightLine(point2, point3)
                    #             print("--------------------------")
                    #             orien = line.getQuaternion()
                    #             pointStop.point.orientation.z = orien[2]
                    #             pointStop.point.orientation.w = orien[3]
                    #             self.msgListPoint.infoStopPoint.append(pointStop)

                    #     elif typeP2 == self.quadraticBezierCurvesMode:
                    #         point1 = Point(data.pathInfo[i].pointOne.position.x, data.pathInfo[i].pointOne.position.y)
                    #         point2 = Point(data.pathInfo[i].pointSecond.position.x, data.pathInfo[i].pointSecond.position.y)
                    #         point3 = Point(data.pathInfo[i+1].pointMid.position.x, data.pathInfo[i+1].pointMid.position.y)

            # tao list point on path
            for i in range(num):
                pathID = data.pathInfo[i].pathID
                if pathID:
                    typePath = data.pathInfo[i].typePath
                    if typePath == self.straightLineMode:
                        pointOne = Point(data.pathInfo[i].pointOne.position.x, data.pathInfo[i].pointOne.position.y)
                        pointSecond = Point(data.pathInfo[i].pointSecond.position.x, data.pathInfo[i].pointSecond.position.y)
                        strLine = StraightLine(pointOne, pointSecond)
                        quat = strLine.getQuaternion()
                        listPoint = strLine.slip(0.01)

                        for j in listPoint:
                            point = PoseStamped()
                            point.header.frame_id = 'map'
                            point.header.stamp = rospy.Time.now()
                            point.pose.position.x = j[0]
                            point.pose.position.y = j[1]
                            point.pose.position.z = 0.
                            point.pose.orientation.w = 1.
                            # print(point)
                            self.msgPath.poses.append(point)

                            pointRQ = PointRequestMove()
                            pointRQ.pathID = pathID
                            pointRQ.typePath = typePath
                            pointRQ.direction = data.pathInfo[i].direction
                            pointRQ.movableZone = data.pathInfo[i].movableZone
                            pointRQ.velocity = data.pathInfo[i].velocity
                            pointRQ.fieldSafety = data.pathInfo[i].fieldSafety
                            # position
                            pointRQ.pose.position.x = j[0]
                            pointRQ.pose.position.y = j[1]
                            # orientation
                            pointRQ.pose.orientation.x = quat[0]
                            pointRQ.pose.orientation.y = quat[1]
                            pointRQ.pose.orientation.z = quat[2]
                            pointRQ.pose.orientation.w = quat[3]

                            self.msgListPoint.infoPoint.append(pointRQ)

                    elif typePath == self.circleMode:
                        pointOne = Point(data.pathInfo[i].pointOne.position.x, data.pathInfo[i].pointOne.position.y)
                        pointSecond = Point(data.pathInfo[i].pointSecond.position.x, data.pathInfo[i].pointSecond.position.y)
                        pointCenter = Point(data.pathInfo[i].pointCenter.position.x, data.pathInfo[i].pointCenter.position.y)
                        radius = data.pathInfo[i].radius
                        circleLine = Circle(pointCenter, radius)
                        listPoint = circleLine.slip(pointOne, pointSecond, 0.01)

                        for j in listPoint:
                            point = PoseStamped()
                            point.header.frame_id = 'map'
                            point.header.stamp = rospy.Time.now()
                            point.pose.position.x = j[0]
                            point.pose.position.y = j[1]
                            point.pose.position.z = 0.
                            point.pose.orientation.w = 1.
                            # print(point)
                            self.msgPath.poses.append(point)

                            pointRQ = PointRequestMove()
                            pointRQ.pathID = pathID
                            pointRQ.typePath = typePath
                            pointRQ.direction = data.pathInfo[i].direction
                            pointRQ.movableZone = data.pathInfo[i].movableZone
                            pointRQ.velocity = data.pathInfo[i].velocity
                            pointRQ.fieldSafety = data.pathInfo[i].fieldSafety
                            # position
                            pointRQ.pose.position.x = j[0]
                            pointRQ.pose.position.y = j[1]
                            # orientation
                            # pointRQ.pose.orientation.x = 1.
                            # pointRQ.pose.orientation.y = 1.
                            # pointRQ.pose.orientation.z = 1.
                            pointRQ.pose.orientation.w = 1.

                            self.msgListPoint.infoPoint.append(pointRQ)

                    elif typePath == self.quadraticBezierCurvesMode:
                        pointOne = Point(data.pathInfo[i].pointOne.position.x, data.pathInfo[i].pointOne.position.y)
                        pointSecond = Point(data.pathInfo[i].pointSecond.position.x, data.pathInfo[i].pointSecond.position.y)
                        pointMid = Point(data.pathInfo[i].pointMid.position.x, data.pathInfo[i].pointMid.position.y)
                        numberPts = data.pathInfo[i].numberPts
                        QBCLine =  QuadraticBezierCurves(pointOne, pointSecond, pointMid, numberPts)

                        listPoint = QBCLine.slip()

                        for j in listPoint:
                            point = PoseStamped()
                            point.header.frame_id = 'map'
                            point.header.stamp = rospy.Time.now()
                            point.pose.position.x = j[0]
                            point.pose.position.y = j[1]
                            point.pose.position.z = 0.
                            point.pose.orientation.w = 1.
                            # print(point)
                            self.msgPath.poses.append(point)

                            pointRQ = PointRequestMove()
                            pointRQ.pathID = pathID
                            pointRQ.typePath = typePath
                            pointRQ.direction = data.pathInfo[i].direction
                            pointRQ.movableZone = data.pathInfo[i].movableZone
                            pointRQ.velocity = data.pathInfo[i].velocity
                            pointRQ.fieldSafety = data.pathInfo[i].fieldSafety
                            # position
                            pointRQ.pose.position.x = j[0]
                            pointRQ.pose.position.y = j[1]
                            # orientation
                            # pointRQ.pose.orientation.x = 1.
                            # pointRQ.pose.orientation.y = 1.
                            # pointRQ.pose.orientation.z = 1.
                            pointRQ.pose.orientation.w = 1.

                            self.msgListPoint.infoPoint.append(pointRQ)

                    elif typePath == self.curverLineMode:
                        pass

                    else:
                        pass

                    self.makePathDone = True

        # print(data.enable, self.msgListPoint.enable)

        self.pubPath.publish(self.msgPath)
        self.pubListPoint.publish(self.msgListPoint)
        self.is_moveRequest = True

    def calculate_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        return sqrt(x*x + y*y)
    
    def resetAll(self):
        del self.msgListPoint.infoPoint[:]
        del self.msgListPoint.infoStopPoint[:]
        del self.msgPath.poses[:]
        self.makePathDone = False

    """
    format 
    typeLine | px1 | py1 | px2 | py2 | R
    """
    def test(self):
        # self.makePath(self.pointOne, self.pointSecond)
        stl = StraightLine(self.pointOne, self.pointSecond)
        print(stl.slip(0.1))

        circle = Circle(Point(1., 1.), 4.)
        print(circle.slip(Point(-3., 1.) ,Point(1., 5.),  1))

    def run(self):
        step = 0
        while not rospy.is_shutdown():    
            if self.is_moveRequest:
                
                self.is_moveRequest = False
            self.rate.sleep()


def main():
    # Start the job threads
    class_1 = MakePathByPoint()
    # class_1.run()
    # class_1.run()
    # Keep the main thread running, otherwise signals are ignored.
    # rospy.spin()

if __name__ == '__main__':
	main()