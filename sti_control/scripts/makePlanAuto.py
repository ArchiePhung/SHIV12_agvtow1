#!/usr/bin/env python3
# license removed for brevity
import numpy as np
import json
from math import sqrt, pow, atan, acos, modf, atan2, fabs, tan, degrees, radians
from math import pi as PI

import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from sti_msgs.msg import PathInfo, PointRequestMove, ListPointRequestMove, LineRequestMove
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Point():
  def __init__(self, _x=0, _y=0):
    self.x = _x
    self.y = _y

class StraightLine():
    def __init__(self, _pointOne=Point(), _pointSecond=Point()):
        self.pointOne = _pointOne
        self.pointSecond = _pointSecond

        self.a = self.pointOne.y - self.pointSecond.y
        self.b = self.pointSecond.x - self.pointOne.x
        self.c = -self.pointOne.x*self.a - self.pointOne.y*self.b

        # print("Make a Straight Line")
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
            # print(self.midPoint.x ,self.midPoint.y)
        # self.numberPts = _numberPts

        self.numberPts = self.findNumberPts()

        self.t = np.array([i*1/self.numberPts for i in range(0,self.numberPts+1)])

        # print("Make a Quadratic Bezier Curves ")

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


class fakePlan():
    def __init__(self):
        self.vel_max = 0.8
        self.QT = []

        self.updateQT()
        # print(self.listTargetneedWaitToStart)
        # print(self.listQTWaitToStart)

    def updateQT(self):
        if len(self.QT) > 0:
            self.QT.clear()

        self.listTargetneedWaitToStart = {}
        self.listQTWaitToStart = {}

        self.dirpath_process = '/home/stivietnam/catkin_ws/src/sti_control/data/process.json'
        with open(self.dirpath_process,'r',encoding='utf-8') as r:
            try:
                self.dataRead_Json = json.load(r)
            except Exception as e:
                print("error: ", e)
                return
                # self.dataRead_Json = json.loads('['+r.read().replace('}{','},{')+']')[0]

        # id wait
        for id_wait in self.dataRead_Json["id_wait"]:
            self.listTargetneedWaitToStart[id_wait["id"]] = id_wait["time_wait"]
        
        # process
        for qt in self.dataRead_Json["process"]:
            line = LineRequestMove()
            line.enable                         = qt["enable"]
            line.target_id                      = qt["target_id"]
            line.target_x                       = qt["target_x"]
            line.target_y                       = qt["target_y"]
            line.target_z                       = qt["target_z"]

            for p in qt["path_info"]:
                path = PathInfo()
                path.pathID                     = p["pathID"]
                path.typePath                   = p["typePath"]
                path.direction                  = p["direction"]

                path.pointOne = Pose()
                path.pointOne.position.x        = p["pointOne"]["x"]
                path.pointOne.position.y        = p["pointOne"]["y"]

                path.pointSecond = Pose()
                path.pointSecond.position.x     = p["pointSecond"]["x"]
                path.pointSecond.position.y     = p["pointSecond"]["y"]

                path.pointMid = Pose()
                path.pointMid.position.x        = p["pointMid"]["x"]
                path.pointMid.position.y        = p["pointMid"]["y"]

                path.velocity                   = (p["velocity"]/100.)*self.vel_max
                path.numberPts                  = p["numberPts"]
                path.movableZone                = p["movableZone"]
                path.fieldSafety                = p["fieldSafety"]

                line.pathInfo.append(path)

            self.QT.append(line)

        self.listQTWaitToStart = self.findQTbyIdTarget(self.listTargetneedWaitToStart)

    def findQTbyIdTarget(self, dic_id):
        lout = {}
        for id in  dic_id:
            for index, qt in enumerate(self.QT):
                if id == qt.target_id:
                    lout[index] = dic_id[id]
                    break

        return lout

    def needWaitTimeToNextProcess(self, qt_now):
        return self.listQTWaitToStart.get(qt_now, -1)
    
    def agvAtQT(self, x_rb, y_rb):
        index = 0
        dmin = 500.

        num = len(self.QT)
        for i in  range(0, num, 1):
            dis_tg = sqrt((self.QT[i].target_x - x_rb)*(self.QT[i].target_x - x_rb) + (self.QT[i].target_y - y_rb)*(self.QT[i].target_y - y_rb))
            if (dis_tg <= 0.1):
                qt = i + 1
                if qt >= num:
                    qt = 0
                return qt

            for ip in self.QT[i].pathInfo:
                if ip.typePath == 1: # duong thang
                    pointOne = Point(ip.pointOne.position.x, ip.pointOne.position.y)
                    pointSecond = Point(ip.pointSecond.position.x, ip.pointSecond.position.y)
                    strLine = StraightLine(pointOne, pointSecond)
                    listPoint = strLine.slip(0.2)
                    for p in listPoint:
                        dis = sqrt((p[0] - x_rb)*(p[0] - x_rb) + (p[1] - y_rb)*(p[1] - y_rb))
                        if dmin > dis:
                            dmin = dis
                            index = i

                elif ip.typePath == 3: # benze
                    pointOne = Point(ip.pointOne.position.x, ip.pointOne.position.y)
                    pointSecond = Point(ip.pointSecond.position.x, ip.pointSecond.position.y)

                    pointOne = Point(ip.pointOne.position.x, ip.pointOne.position.y)
                    pointSecond = Point(ip.pointSecond.position.x, ip.pointSecond.position.y)
                    pointMid = Point(ip.pointMid.position.x, ip.pointMid.position.y)
                    numberPts = ip.numberPts
                    QBCLine =  QuadraticBezierCurves(pointOne, pointSecond, pointMid, numberPts)
                    listPoint = QBCLine.slip()
                    for p in listPoint:
                        dis = sqrt((p[0] - x_rb)*(p[0] - x_rb) + (p[1] - y_rb)*(p[1] - y_rb))
                        if dmin > dis:
                            dmin = dis
                            index = i

        return index

def main():
    program = fakePlan()

if __name__ == '__main__':
    main()




