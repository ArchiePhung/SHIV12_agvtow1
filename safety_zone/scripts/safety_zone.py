#!/usr/bin/env python

from visualization_msgs.msg import Marker , MarkerArray
from std_msgs.msg import Empty , ColorRGBA
from geometry_msgs.msg import  Pose , Point
from sensor_msgs.msg import PointCloud2, LaserScan
from decimal import Decimal
from sti_msgs.msg import *
from math import atan2, sin, cos, sqrt, fabs
from math import pi as PI
import rospy
import time
import threading
import signal

class safety_zone():
    def __init__(self):
        
        rospy.init_node('safety_zone', anonymous = True)
        self.rate = rospy.Rate(20)

        #get param 
        self.dx_robot = rospy.get_param('~dx_robot',0.5)
        self.dy_robot = rospy.get_param('~dy_robot',0.26)

        self.dx1_head = rospy.get_param('~dx1_head',0.5)
        self.dx2_head = rospy.get_param('~dx2_head',1.0)
        self.dx3_head = rospy.get_param('~dx3_head',2.0)

        self.dx1_behind = rospy.get_param('~dx1_behind',0.5)
        self.dx2_behind = rospy.get_param('~dx2_behind',1.0)
        self.dx3_behind = rospy.get_param('~dx3_behind',2.0)

        self.dx_shelves = rospy.get_param('~dx_shelves',0)
        self.dy_shelves = rospy.get_param('~dy_shelves',0.6)

        self.do_phan_giai_goc_quet = rospy.get_param('~do_phan_giai_goc_quet', 1)

        # topic pub-sub
        rospy.Subscriber("/scan", LaserScan, self.call_sub)
        self.zone_lidar_2head_pub = rospy.Publisher('safety_zone',Zone_lidar_2head, queue_size=10) 


        self.zone_2head = Zone_lidar_2head()
        self.d_scan_all = LaserScan()
        self.is_scan_all = False

        self.range_max = 0.0
        self.angle_head_from = -PI/2.0
        self.angle_head_to = PI/2.0
        self.angle_behind_from = PI/2.0
        self.angle_behind_to = -PI/2.0


    def call_sub(self,data):
        self.d_scan_all = data
        self.range_max = float((float(self.d_scan_all.angle_max) - float(self.d_scan_all.angle_min))/float(self.d_scan_all.angle_increment))
        if self.is_scan_all == False :
            self.is_scan_all = True

        # print(self.d_scan_all.ranges[0])


    def check_zone_2head(self,angle_from,angle_to,dx1,dx2,dx3,n_res):
        dem_z1_t = dem_z2_t = dem_z3_t = 0
        number_from = number_to = 0
        x_point = y_point = 0.0
        # range_max = float((float(self.d_scan_all.angle_max) - float(self.d_scan_all.angle_min))/float(self.d_scan_all.angle_increment))
        # print(self.range_max)

        if angle_from <= self.d_scan_all.angle_min :
            # angle_from = self.d_scan_all.angle_min
            number_from = 0
        else:
            number_from = int(round((fabs(self.d_scan_all.angle_min) + angle_from)/self.d_scan_all.angle_increment))

        if angle_to >= self.d_scan_all.angle_max : 
            # angle_to = self.d_scan_all.angle_max
            number_to = int(round(self.range_max))
        else:
            number_to = int(round((fabs(self.d_scan_all.angle_min) + angle_to)/self.d_scan_all.angle_increment))

        for i in range(number_from, number_to, n_res):
            angle_cur = self.d_scan_all.angle_min + i*self.d_scan_all.angle_increment

            # print(angle_cur)
            if fabs(angle_cur) > PI/2.0:
                if angle_cur > 0:
                    x_point = -cos(fabs(PI - angle_cur))*self.d_scan_all.ranges[i]
                    y_point = -sin(fabs(PI - angle_cur))*self.d_scan_all.ranges[i]
                else:
                    x_point = -cos(fabs(PI - angle_cur))*self.d_scan_all.ranges[i]
                    y_point = sin(fabs(PI - angle_cur))*self.d_scan_all.ranges[i]

            else:
                if angle_cur > 0:
                    x_point = cos(fabs(angle_cur))*self.d_scan_all.ranges[i]
                    y_point = -sin(fabs(angle_cur))*self.d_scan_all.ranges[i]
                else:
                    x_point = cos(fabs(angle_cur))*self.d_scan_all.ranges[i]
                    y_point = sin(fabs(angle_cur))*self.d_scan_all.ranges[i]

            # if angle_cur >= 0.0 and angle_cur < PI/2.0:
            #     x_point = -cos(angle_cur)*self.d_scan_all.ranges[i]
            #     y_point = -sin(angle_cur)*self.d_scan_all.ranges[i]
            # elif angle_cur >= PI/2.0 and angle_cur < PI:
            #     x_point = cos(PI -  angle_cur)*self.d_scan_all.ranges[i]
            #     y_point = -sin(PI - angle_cur)*self.d_scan_all.ranges[i]
            # elif angle_cur >= PI and angle_cur < (3.0*PI)/2.0:
            #     x_point = cos(angle_cur - PI)*self.d_scan_all.ranges[i]
            #     y_point = sin(angle_cur - PI)*self.d_scan_all.ranges[i]
            # else:
            #     x_point = -cos(2.0*PI - angle_cur)*self.d_scan_all.ranges[i]
            #     y_point = sin(2.0*PI - angle_cur)*self.d_scan_all.ranges[i]


            # print('x_point = %lf, y_point = %lf' %(x_point,y_point))

            if fabs(x_point) < dx1 + self.dx_robot and fabs(x_point) > self.dx_robot and fabs(y_point) < self.dy_shelves :
                dem_z1_t = dem_z1_t + 1
                # if dem_z1_t >= 5:
                #     break

            if fabs(x_point) < dx2 + self.dx_robot and fabs(x_point) > dx1 + self.dx_robot and fabs(y_point) < self.dy_shelves :
                dem_z2_t = dem_z2_t + 1

            if fabs(x_point) < dx3 + self.dx_robot and fabs(x_point) > dx2 + self.dx_robot and fabs(y_point) < self.dy_shelves :
                dem_z3_t = dem_z3_t + 1

        return dem_z1_t, dem_z2_t, dem_z3_t
                

    def raa(self):
        while not rospy.is_shutdown() :
            np_ahead_z1 = np_ahead_z2 = np_ahead_z3 = 0
            np_behind_z1_all = np_behind_z2_all = np_behind_z3_all = 0
            np_behind_z1_1 = np_behind_z2_1 = np_behind_z3_1 = 0
            np_behind_z1_2 = np_behind_z2_2 = np_behind_z3_2 = 0
            if self.is_scan_all == True:
                self.is_scan_all = False
                # check phia truoc
                np_ahead_z1, np_ahead_z2, np_ahead_z3 = self.check_zone_2head(self.angle_head_from,\
                                                                                self.angle_head_to,\
                                                                                self.dx1_head,\
                                                                                self.dx2_head,\
                                                                                self.dx3_head,\
                                                                                self.do_phan_giai_goc_quet)

                # print(np_ahead_z1,np_ahead_z2,np_ahead_z3)

                if np_ahead_z1 > 0:
                    np_ahead_z1 = 0
                    self.zone_2head.zone_ahead = 1

                elif np_ahead_z2 > 0:
                    np_ahead_z2 = 0
                    self.zone_2head.zone_ahead = 2

                elif np_ahead_z3 > 0:
                    np_ahead_z3 = 0
                    self.zone_2head.zone_ahead = 3

                else:
                    self.zone_2head.zone_ahead = 0

                # check phia sau
                np_behind_z1_1, np_behind_z2_1, np_behind_z3_1 = self.check_zone_2head(self.d_scan_all.angle_min,\
                                                                                self.angle_behind_to,\
                                                                                self.dx1_behind,\
                                                                                self.dx2_behind,\
                                                                                self.dx3_behind,\
                                                                                self.do_phan_giai_goc_quet)

                np_behind_z1_2, np_behind_z2_2, np_behind_z3_2 = self.check_zone_2head(self.angle_behind_from,\
                                                                                self.d_scan_all.angle_max,\
                                                                                self.dx1_behind,\
                                                                                self.dx2_behind,\
                                                                                self.dx3_behind,\
                                                                                self.do_phan_giai_goc_quet)

                np_behind_z1_all = np_behind_z1_1 + np_behind_z1_2
                np_behind_z2_all = np_behind_z2_1 + np_behind_z2_2
                np_behind_z3_all = np_behind_z3_1 + np_behind_z3_2

                if np_behind_z1_all > 2:
                    np_behind_z1_all = 0
                    self.zone_2head.zone_behind = 1

                elif np_behind_z2_all > 1:
                    np_behind_z2_all = 0
                    self.zone_2head.zone_behind = 2

                elif np_behind_z3_all > 1:
                    np_behind_z3_all = 0
                    self.zone_2head.zone_behind = 3

                else:
                    self.zone_2head.zone_behind = 0

                # print(self.zone_2head.zone_ahead)
                self.zone_lidar_2head_pub.publish(self.zone_2head) 

            self.rate.sleep()
        # print('Thread #%s stopped' % self.threadID)

def main():
    
    try:
        m = safety_zone()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()
