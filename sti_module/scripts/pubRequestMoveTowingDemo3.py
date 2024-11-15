#!/usr/bin/env python3
# license removed for brevity
import roslib
import sys
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from sti_msgs.msg import PathInfo, PointRequestMove, ListPointRequestMove, LineRequestMove

from math import sin, cos, asin, tan, atan, degrees, radians, sqrt, fabs, acos, atan2
from math import pi as PI

def calAngleThreePoint(x1, y1, x2, y2, x3, y3):
    dx1 = x1 - x2
    dy1 = y1 - y2
    dx2 = x3 - x2
    dy2 = y3 - y2
    c_goc = (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-12)
    goc = acos(c_goc)
    
    return goc

def fnCalcDistPoints(x1, x2, y1, y2):
    return sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

def funcalptduongthang(X_s, Y_s, X_f, Y_f):
    _a = Y_s - Y_f
    _b = X_f - X_s
    _c = -X_s*_a -Y_s*_b
    return _a, _b, _c

def ptduongthangvuonggoc(a, b, _x, _y):
    return a, b, (-1)*a*_x - b*_y

def funcalPointTT(a_qd, b_qd, c_qd, _d, _x, _y, _x_s, _y_s, _dis):
    X_c = Y_c =  X_c1 = Y_c1 = X_c2 = Y_c2 = 0.0
    if b_qd == 0.0:
        X_c1 = X_c2 = -c_qd/a_qd
        Y_c1 = -sqrt(_d*_d - (X_c1 - _x)*(X_c1 - _x)) + _y
        Y_c2 = sqrt(_d*_d - (X_c2 - _x)*(X_c2 - _x)) + _y
        
    else:
        la = (1.0 + (a_qd/b_qd)*(a_qd/b_qd))
        lb = -2.0*(_x - (a_qd/b_qd)*((c_qd/b_qd) + _y))
        lc = _x*_x + ((c_qd/b_qd) + _y)*((c_qd/b_qd) + _y) - _d*_d
        denlta = lb*lb - 4.0*la*lc
        # print(la,lb,lc,denlta)

        X_c1 = (-lb + sqrt(denlta))/(2.0*la)
        X_c2 = (-lb - sqrt(denlta))/(2.0*la)

        Y_c1 = (-c_qd - a_qd*X_c1)/b_qd
        Y_c2 = (-c_qd - a_qd*X_c2)/b_qd
        
    disC1 = fnCalcDistPoints(_x_s, X_c1, _y_s, Y_c1)
    # disC2 = self.fnCalcDistPoints(_x_s, X_c2, _y_s, Y_c2)
    
    if disC1 < _dis:
        X_c = X_c1
        Y_c = Y_c1
        
    else:
        X_c = X_c2
        Y_c = Y_c2
    
    return X_c, Y_c

def funcalCircle(x1, y1, x2, y2, x3, y3):
        X_c = 0
        Y_c = 0
        R = 1.
        # tim pt duong thang 1:
        a1, b1, c1 = funcalptduongthang(x1, y1, x2, y2)
        a2, b2, c2 = funcalptduongthang(x2, y2, x3, y3)
        
        # tim ban kinh duong tron:
        _theta = calAngleThreePoint(x1, y1, x2, y2, x3, y3)
        d1 = fnCalcDistPoints(x1, x2, y1, y2)
        d2 = fnCalcDistPoints(x2, x3, y2, y3)
        dmin = min(d1, d2)
        rospy.logwarn("d1 = %s |d2 = %s |dmin = %s", d1, d2 , dmin)
        
        r = (dmin/2)*tan(_theta/2)
        # R = constrain(r, MinRadiusCircle, MaxRadiusCircle)
        
        # tim 2 diem tiep tuyen:
        _d = R/tan(_theta/2)
        Xc1, Yc1 = funcalPointTT(a1, b1, c1, _d, x2, y2, x1, y1, d1)
        Xc2, Yc2 = funcalPointTT(a2, b2, c2, _d, x2, y2, x3, y3, d2)
        
        # tim tam duong tron:
        avg1, bvg1, cvg1 = ptduongthangvuonggoc(b1, (-1)*a1, Xc1, Yc1)
        avg2, bvg2, cvg2 = ptduongthangvuonggoc(b2, (-1)*a2, Xc2, Yc2)
        
        if bvg1 == 0.0:
            X_c = -cvg1/avg1
            Y_c = (-cvg2 - avg2*X_c)/bvg2
            
        elif bvg2 == 0.0:
            X_c = -cvg2/avg2
            Y_c = (-cvg1 - avg1*X_c)/bvg1
            
        else:
            X_c = ((cvg1/bvg1)-(cvg2/bvg2))/((avg2/bvg2)-(avg1/bvg1))
            Y_c = (-cvg1 - avg1*X_c)/bvg1
        
        return X_c, Y_c, R, Xc1, Yc1, Xc2, Yc2, _d

def talker():
    pub = rospy.Publisher('/request_move', LineRequestMove, queue_size=10)
    rospy.init_node('pub_requestMove', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    msg = LineRequestMove()
    msg.enable = 1
    msg.target_x = -1.012
    msg.target_y = -0.189
    msg.target_z = 0.

    path = PathInfo()
    path.pathID = 8
    path.typePath = 1
    path.direction = 1
    path.pointOne = Pose()
    path.pointOne.position.x = -1.012
    path.pointOne.position.y = 8.249
    path.pointSecond = Pose()
    path.pointSecond.position.x = -1.012
    path.pointSecond.position.y = -0.189
    path.velocity = 0.4
    path.movableZone = 1.2
    msg.pathInfo.append(path)

    while not rospy.is_shutdown():
        rospy.loginfo("in procress PUB")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass