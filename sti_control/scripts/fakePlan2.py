#!/usr/bin/env python3
# license removed for brevity
import numpy as np
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

class fakePlan():
    def __init__(self):
        self.allQT = []
        self.listQTWaitToStart = {1: 30., 2: 60.}

        self.QT1 = []

        self.QT1_1 = LineRequestMove()
        self.QT1_1.enable = 1
        self.QT1_1.target_x = -121.02
        self.QT1_1.target_y = -7.088
        self.QT1_1.target_z = 0.

        path = PathInfo()
        path.pathID = 1
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -126.75
        path.pointOne.position.y = -6.951
        path.pointSecond = Pose()
        path.pointSecond.position.x = -126.802
        path.pointSecond.position.y = -11.093
        path.velocity = 0.4
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_1.pathInfo.append(path)

        # path  Benze 
        path = PathInfo()
        path.pathID = 2
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -126.802
        path.pointOne.position.y = -11.093
        path.pointSecond = Pose()
        path.pointSecond.position.x = -123.75
        path.pointSecond.position.y = -12.94
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -126.802
        path.pointMid.position.y = -12.94
        path.fieldSafety = 0
        self.QT1_1.pathInfo.append(path)

        # path  Benze 
        path = PathInfo()
        path.pathID = 3
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -123.75
        path.pointOne.position.y = -12.94
        path.pointSecond = Pose()
        path.pointSecond.position.x = -120.842
        path.pointSecond.position.y = -11.103
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -120.842
        path.pointMid.position.y = -12.94
        path.fieldSafety = 0
        self.QT1_1.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 4
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -120.842
        path.pointOne.position.y = -11.103
        path.pointSecond = Pose()
        path.pointSecond.position.x = -121.02
        path.pointSecond.position.y = -7.088
        path.velocity = 0.3
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_1.pathInfo.append(path)

        self.QT1.append(self.QT1_1)

        self.QT1_2 = LineRequestMove()
        self.QT1_2.enable = 1
        self.QT1_2.target_x = -10.406
        self.QT1_2.target_y = -1.255
        self.QT1_2.target_z = 0.

        path = PathInfo()
        path.pathID = 1
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -121.02
        path.pointOne.position.y = -7.088
        path.pointSecond = Pose()
        path.pointSecond.position.x = -121.118
        path.pointSecond.position.y = -3.085
        path.velocity = 0.3
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        # path  Benze 
        path = PathInfo()
        path.pathID = 2
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -121.118
        path.pointOne.position.y = -3.085
        path.pointSecond = Pose()
        path.pointSecond.position.x = -119.67
        path.pointSecond.position.y = -1.9
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -121.118
        path.pointMid.position.y = -1.9
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 3
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -119.67
        path.pointOne.position.y = -1.9
        path.pointSecond = Pose()
        path.pointSecond.position.x = -117.46
        path.pointSecond.position.y = -1.856
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 4
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -117.46
        path.pointOne.position.y = -1.856
        path.pointSecond = Pose()
        path.pointSecond.position.x = -111.874
        path.pointSecond.position.y = -1.912
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 5
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -111.874
        path.pointOne.position.y = -1.912
        path.pointSecond = Pose()
        path.pointSecond.position.x = -101.602
        path.pointSecond.position.y = -1.867
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 6
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -101.602
        path.pointOne.position.y = -1.867
        path.pointSecond = Pose()
        path.pointSecond.position.x = -92.679
        path.pointSecond.position.y = -1.85
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 7
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -92.679
        path.pointOne.position.y = -1.85
        path.pointSecond = Pose()
        path.pointSecond.position.x = -83.75
        path.pointSecond.position.y = -1.823
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 8
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -83.75
        path.pointOne.position.y = -1.823
        path.pointSecond = Pose()
        path.pointSecond.position.x = -75.096
        path.pointSecond.position.y = -1.734
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 9
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -75.096
        path.pointOne.position.y = -1.734
        path.pointSecond = Pose()
        path.pointSecond.position.x = -66.012
        path.pointSecond.position.y = -1.698
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 10
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -66.012
        path.pointOne.position.y = -1.698
        path.pointSecond = Pose()
        path.pointSecond.position.x = -59.603
        path.pointSecond.position.y = -1.63
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 11
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -59.603
        path.pointOne.position.y = -1.63
        path.pointSecond = Pose()
        path.pointSecond.position.x = -41.614
        path.pointSecond.position.y = -1.478
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 12
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -41.614
        path.pointOne.position.y = -1.478
        path.pointSecond = Pose()
        path.pointSecond.position.x = -36.036
        path.pointSecond.position.y = -1.442
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 13
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -36.036
        path.pointOne.position.y = -1.442
        path.pointSecond = Pose()
        path.pointSecond.position.x = -21.425
        path.pointSecond.position.y = -1.426
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 14
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -21.425
        path.pointOne.position.y = -1.426
        path.pointSecond = Pose()
        path.pointSecond.position.x = -10.406
        path.pointSecond.position.y = -1.255
        path.velocity = 0.4
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_2.pathInfo.append(path)

        self.QT1.append(self.QT1_2)

        self.QT1_3 = LineRequestMove()
        self.QT1_3.enable = 1
        self.QT1_3.target_x = -2.0
        self.QT1_3.target_y = -1.173
        self.QT1_3.target_z = 0.

        path = PathInfo()
        path.pathID = 1
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -10.406
        path.pointOne.position.y = -1.255
        path.pointSecond = Pose()
        path.pointSecond.position.x = -2.0
        path.pointSecond.position.y = -1.173
        path.velocity = 0.4
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_3.pathInfo.append(path)

        self.QT1.append(self.QT1_3)


        self.QT1_4 = LineRequestMove()
        self.QT1_4.enable = 1
        self.QT1_4.target_x = -0.52
        self.QT1_4.target_y = 9.648
        self.QT1_4.target_z = 0.

        path = PathInfo()
        path.pathID = 1
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -2.0
        path.pointOne.position.y = -1.173
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.575
        path.pointSecond.position.y = -1.133
        path.velocity = 0.25
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_4.pathInfo.append(path)

        # path  Benze 
        path = PathInfo()
        path.pathID = 2
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.575
        path.pointOne.position.y = -1.133
        path.pointSecond = Pose()
        path.pointSecond.position.x = 1.055
        path.pointSecond.position.y = -0.285
        path.velocity = 0.15
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = 0.8
        path.pointMid.position.y = -1
        path.fieldSafety = 0
        self.QT1_4.pathInfo.append(path)

        # path  Benze 
        path = PathInfo()
        path.pathID = 3
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = 1.055
        path.pointOne.position.y = -0.285
        path.pointSecond = Pose()
        path.pointSecond.position.x = 0.518
        path.pointSecond.position.y = 1.132
        path.velocity = 0.15
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = 0.85
        path.pointMid.position.y = 0.65
        path.fieldSafety = 0
        self.QT1_4.pathInfo.append(path)

        # path  Benze 
        path = PathInfo()
        path.pathID = 4
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = 0.518
        path.pointOne.position.y = 1.132
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.236
        path.pointSecond.position.y = 1.777
        path.velocity = 0.15
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = 0.
        path.pointMid.position.y = 1.2
        path.fieldSafety = 0
        self.QT1_4.pathInfo.append(path)

        # path  Benze 
        path = PathInfo()
        path.pathID = 5
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.236
        path.pointOne.position.y = 1.777
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.47
        path.pointSecond.position.y = 3.142
        path.velocity = 0.15
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -0.47
        path.pointMid.position.y = 2.5
        path.fieldSafety = 0
        self.QT1_4.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 6
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.47
        path.pointOne.position.y = 3.142
        path.pointSecond = Pose()
        path.pointSecond.position.x =-0.47
        path.pointSecond.position.y = 5.264
        path.velocity = 0.2
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_4.pathInfo.append(path)

        # path = PathInfo()
        # path.pathID = 7
        # path.typePath = 1
        # path.direction = 1
        # path.pointOne = Pose()
        # path.pointOne.position.x = -0.47
        # path.pointOne.position.y = 3.142
        # path.pointSecond = Pose()
        # path.pointSecond.position.x = -0.47
        # path.pointSecond.position.y = 5.264
        # path.velocity = 0.2
        # path.movableZone = 1.2
        # path.fieldSafety = 0
        # self.QT1_4.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 8
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.47
        path.pointOne.position.y = 5.264
        path.pointSecond = Pose()
        path.pointSecond.position.x =-0.475
        path.pointSecond.position.y = 6.562
        path.velocity = 0.2
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_4.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 9
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.475
        path.pointOne.position.y = 6.562
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.493
        path.pointSecond.position.y = 8.794
        path.velocity = 0.2
        path.movableZone = 1.2
        path.fieldSafety = 1
        self.QT1_4.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 10
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.493
        path.pointOne.position.y = 8.794
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.52
        path.pointSecond.position.y = 9.648
        path.velocity = 0.2
        path.movableZone = 1.2
        path.fieldSafety = 1
        self.QT1_4.pathInfo.append(path)

        self.QT1.append(self.QT1_4)

        # -------------------------------------

        self.QT1_5 = LineRequestMove()
        self.QT1_5.enable = 1
        self.QT1_5.target_x = -0.591
        self.QT1_5.target_y = 22.444
        self.QT1_5.target_z = 0.

        path = PathInfo()
        path.pathID = 1
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.52
        path.pointOne.position.y = 9.648
        path.pointSecond = Pose()
        path.pointSecond.position.x =-0.5
        path.pointSecond.position.y = 11.734
        path.velocity = 0.2
        path.movableZone = 1.2
        path.fieldSafety = 1
        self.QT1_5.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 2
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.5
        path.pointOne.position.y = 11.734
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.5
        path.pointSecond.position.y = 15.674
        path.velocity = 0.2
        path.movableZone = 1.2
        path.fieldSafety = 1
        self.QT1_5.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 3
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.5
        path.pointOne.position.y = 15.674
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.538
        path.pointSecond.position.y = 17.647
        path.velocity = 0.2
        path.movableZone = 1.2
        path.fieldSafety = 1
        self.QT1_5.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 4
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.538
        path.pointOne.position.y = 17.647
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.566
        path.pointSecond.position.y = 19.655
        path.velocity = 0.2
        path.movableZone = 1.2
        path.fieldSafety = 1
        self.QT1_5.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 11
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.566
        path.pointOne.position.y = 19.655
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.591
        path.pointSecond.position.y = 22.444
        path.velocity = 0.2
        path.movableZone = 1.2
        path.fieldSafety = 1
        self.QT1_5.pathInfo.append(path)

        self.QT1.append(self.QT1_5)

        # -------------------

        self.QT1_6 = LineRequestMove()
        self.QT1_6.enable = 1
        self.QT1_6.target_x = -126.75
        self.QT1_6.target_y = -6.951
        self.QT1_6.target_z = 0.

        path = PathInfo()
        path.pathID = 1
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.591
        path.pointOne.position.y = 22.444
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.553
        path.pointSecond.position.y = 24.281
        path.velocity = 0.5
        path.movableZone = 1.2
        path.fieldSafety = 1
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 2
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.553
        path.pointOne.position.y = 24.281
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.541
        path.pointSecond.position.y = 26.074
        path.velocity = 0.5
        path.movableZone = 1.2
        path.fieldSafety = 1
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 3
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.541
        path.pointOne.position.y = 26.074
        path.pointSecond = Pose()
        path.pointSecond.position.x = -0.521
        path.pointSecond.position.y = 28.224
        path.velocity = 0.4
        path.movableZone = 1.2
        path.fieldSafety = 1
        self.QT1_6.pathInfo.append(path)


        # path  Benze 
        path = PathInfo()
        path.pathID = 4
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -0.521
        path.pointOne.position.y = 28.224
        path.pointSecond = Pose()
        path.pointSecond.position.x = -1.916
        path.pointSecond.position.y = 30.013
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -0.521
        path.pointMid.position.y = 30.013
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 5
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -1.916
        path.pointOne.position.y = 30.013
        path.pointSecond = Pose()
        path.pointSecond.position.x = -7.46
        path.pointSecond.position.y = 29.859
        path.velocity = 0.4
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 6
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -7.46
        path.pointOne.position.y = 29.859
        path.pointSecond = Pose()
        path.pointSecond.position.x = -12.926
        path.pointSecond.position.y = 29.79
        path.velocity = 0.5
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 7
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -7.46
        path.pointOne.position.y = 29.859
        path.pointSecond = Pose()
        path.pointSecond.position.x = -16.79
        path.pointSecond.position.y = 29.718
        path.velocity = 0.4
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        # path  Benze 
        path = PathInfo()
        path.pathID = 8
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -16.79
        path.pointOne.position.y = 29.718
        path.pointSecond = Pose()
        path.pointSecond.position.x = -18.793
        path.pointSecond.position.y = 27.59
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -18.793
        path.pointMid.position.y = 29.718
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 9
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -18.793
        path.pointOne.position.y = 27.59
        path.pointSecond = Pose()
        path.pointSecond.position.x = -18.708
        path.pointSecond.position.y = 20.426
        path.velocity = 0.4
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 10
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -18.708
        path.pointOne.position.y = 20.426
        path.pointSecond = Pose()
        path.pointSecond.position.x = -18.581
        path.pointSecond.position.y = 11.545
        path.velocity = 0.5
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 11
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -18.581
        path.pointOne.position.y = 11.545
        path.pointSecond = Pose()
        path.pointSecond.position.x = -18.483
        path.pointSecond.position.y = 2.392
        path.velocity = 0.4
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        # path  Benze 
        path = PathInfo()
        path.pathID = 12
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -18.483
        path.pointOne.position.y = 2.392
        path.pointSecond = Pose()
        path.pointSecond.position.x = -20.958
        path.pointSecond.position.y = 0.354
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -18.483
        path.pointMid.position.y = 0.354
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 13
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -20.958
        path.pointOne.position.y = 0.354
        path.pointSecond = Pose()
        path.pointSecond.position.x = -29.995
        path.pointSecond.position.y = 0.35
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 14
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -29.995
        path.pointOne.position.y = 0.35
        path.pointSecond = Pose()
        path.pointSecond.position.x = -41.276
        path.pointSecond.position.y = 0.32
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 15
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -41.276
        path.pointOne.position.y = 0.32
        path.pointSecond = Pose()
        path.pointSecond.position.x = -53.495
        path.pointSecond.position.y = 0.2
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 16
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -53.495
        path.pointOne.position.y = 0.3
        path.pointSecond = Pose()
        path.pointSecond.position.x = -63.645
        path.pointSecond.position.y = 0.17
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 17
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -63.645
        path.pointOne.position.y = 0.17
        path.pointSecond = Pose()
        path.pointSecond.position.x = -76.374
        path.pointSecond.position.y = 0.092
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 18
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -76.374
        path.pointOne.position.y = 0.092
        path.pointSecond = Pose()
        path.pointSecond.position.x = -83.609
        path.pointSecond.position.y = 0.04
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 19
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -83.609
        path.pointOne.position.y = 0.04
        path.pointSecond = Pose()
        path.pointSecond.position.x = -93.7
        path.pointSecond.position.y = 0.0
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 20
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -93.7
        path.pointOne.position.y = 0.0
        path.pointSecond = Pose()
        path.pointSecond.position.x = -103.488
        path.pointSecond.position.y = -0.039
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 21
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -103.488
        path.pointOne.position.y = -0.039
        path.pointSecond = Pose()
        path.pointSecond.position.x = -113.36
        path.pointSecond.position.y = -0.058
        path.velocity = 0.65
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 22
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -113.36
        path.pointOne.position.y = -0.058
        path.pointSecond = Pose()
        path.pointSecond.position.x = -123.588
        path.pointSecond.position.y = -0.074
        path.velocity = 0.4
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        # path  Benze 
        path = PathInfo()
        path.pathID = 23
        path.typePath = 3
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -123.588
        path.pointOne.position.y = -0.074
        path.pointSecond = Pose()
        path.pointSecond.position.x = -126.82
        path.pointSecond.position.y = -1.94
        path.velocity = 0.2
        path.numberPts = 150
        path.movableZone = 1.2
        path.pointMid = Pose()
        path.pointMid.position.x = -126.82
        path.pointMid.position.y = -0.074
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 24
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -126.82
        path.pointOne.position.y = -1.94
        path.pointSecond = Pose()
        path.pointSecond.position.x = -126.703
        path.pointSecond.position.y = -4.42
        path.velocity = 0.4
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        path = PathInfo()
        path.pathID = 25
        path.typePath = 1
        path.direction = 1
        path.pointOne = Pose()
        path.pointOne.position.x = -126.703
        path.pointOne.position.y = -4.42
        path.pointSecond = Pose()
        path.pointSecond.position.x = -126.75
        path.pointSecond.position.y = -6.951
        path.velocity = 0.3
        path.movableZone = 1.2
        path.fieldSafety = 0
        self.QT1_6.pathInfo.append(path)

        self.QT1.append(self.QT1_6)

    def needWaitTimeToNextProcess(self, qt_now):
        return self.listQTWaitToStart.get(qt_now, -1)
    
    def agvAtQT(self, x_rb, y_rb):
        index = 0
        dmin = 500.

        num = len(self.QT1)
        for i in  range(0, num, 1):
            dis_tg = sqrt((self.QT1[i].target_x - x_rb)*(self.QT1[i].target_x - x_rb) + (self.QT1[i].target_y - y_rb)*(self.QT1[i].target_y - y_rb))
            if (dis_tg < 0.05):
                qt = i + 1
                if qt >= num:
                    qt = 0
                return qt

            for ip in self.QT1[i].pathInfo:
                pointOne = Point(ip.pointOne.position.x, ip.pointOne.position.y)
                pointSecond = Point(ip.pointSecond.position.x, ip.pointSecond.position.y)
                strLine = StraightLine(pointOne, pointSecond)
                listPoint = strLine.slip(0.2)
                for p in listPoint:
                    dis = sqrt((p[0] - x_rb)*(p[0] - x_rb) + (p[1] - y_rb)*(p[1] - y_rb))
                    if dmin > dis:
                        dmin = dis
                        index = i

        return index



