#!/usr/bin/env python3

from visualization_msgs.msg import Marker , MarkerArray
from std_msgs.msg import Empty , ColorRGBA
from geometry_msgs.msg import  Pose , Point
from sensor_msgs.msg import PointCloud2, LaserScan
from decimal import Decimal
from message_pkg.msg import Safety_zone
from sti_msgs.msg import Zone_lidar_2head

from math import atan2, sin, cos, sqrt, fabs
from math import pi as PI
import rospy
import time
import threading
import signal

class safetyZone():
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

        self.do_phan_giai_goc_quet = rospy.get_param('~resolution_angle', 1)
        self.zone_circle_robot = rospy.get_param('~zone_circle_robot', 0.75)
        self.dy_circle = rospy.get_param('~dy_circle', 0.55)

        # topic pub-sub
        rospy.Subscriber("/scan", LaserScan, self.call_sub)
        self.pub_zone = rospy.Publisher('/safety_zone', Zone_lidar_2head, queue_size= 10) 
        
        self.zone_2head = Zone_lidar_2head()
        self.d_scan_all = LaserScan()
        self.is_scan_all = False

        self.range_max = 0.0
        self.angle_from = -PI/2.0
        self.angle_to = PI/2.0
        self.angle_behind_from = PI/2.0
        self.angle_behind_to = -PI/2.0

    def call_sub(self,data):
        self.d_scan_all = data
        self.range_max = float((float(self.d_scan_all.angle_max) - float(self.d_scan_all.angle_min))/float(self.d_scan_all.angle_increment))
        if self.is_scan_all == False :
            self.is_scan_all = True
        # print(self.d_scan_all.ranges[0])

    def check_zone_2head(self,angle_from,angle_to,dx1_t,dx2_t,dx3_t,dx1_s,dx2_s,dx3_s,n_res):
        dem_z1_t = dem_z2_t = dem_z3_t = 0
        dem_z1_s = dem_z2_s = dem_z3_s = 0
        dem_circle_point_beside = 0
        dem_circle_point_ahead_behind = 0
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

            # check vung sau
            if fabs(angle_cur) > PI/2.0:
                if angle_cur > 0:
                    x_point = -cos(fabs(PI - angle_cur))*self.d_scan_all.ranges[i]
                    y_point = -sin(fabs(PI - angle_cur))*self.d_scan_all.ranges[i]
                else:
                    x_point = -cos(fabs(PI - angle_cur))*self.d_scan_all.ranges[i]
                    y_point = sin(fabs(PI - angle_cur))*self.d_scan_all.ranges[i]

                if fabs(x_point) < dx1_s + self.dx_robot and fabs(x_point) > self.dx_robot and fabs(y_point) < self.dy_shelves :
                    dem_z1_s = dem_z1_s + 1

                if fabs(x_point) < dx2_s + self.dx_robot and fabs(x_point) > dx1_s + self.dx_robot and fabs(y_point) < self.dy_shelves :
                    dem_z2_s = dem_z2_s + 1

                if fabs(x_point) < dx3_s + self.dx_robot and fabs(x_point) > dx2_s + self.dx_robot and fabs(y_point) < self.dy_shelves :
                    dem_z3_s = dem_z3_s + 1    
            # check vung truoc
            else:
                if angle_cur > 0:
                    x_point = cos(fabs(angle_cur))*self.d_scan_all.ranges[i]
                    y_point = -sin(fabs(angle_cur))*self.d_scan_all.ranges[i]
                else:
                    x_point = cos(fabs(angle_cur))*self.d_scan_all.ranges[i]
                    y_point = sin(fabs(angle_cur))*self.d_scan_all.ranges[i]

                if fabs(x_point) < dx1_t + self.dx_robot and fabs(x_point) > self.dx_robot and fabs(y_point) < self.dy_shelves :
                    dem_z1_t = dem_z1_t + 1

                if fabs(x_point) < dx2_t + self.dx_robot and fabs(x_point) > dx1_t + self.dx_robot and fabs(y_point) < self.dy_shelves :
                    dem_z2_t = dem_z2_t + 1

                if fabs(x_point) < dx3_t + self.dx_robot and fabs(x_point) > dx2_t + self.dx_robot and fabs(y_point) < self.dy_shelves :
                    dem_z3_t = dem_z3_t + 1 

            if self.d_scan_all.ranges[i] > 0.0 and self.d_scan_all.ranges[i] < self.zone_circle_robot and fabs(y_point) > self.dy_circle :
                dem_circle_point_beside = dem_circle_point_beside + 1 
                # print('x_point = %lf, y_point = %lf' %(x_point,y_point))
            
            if self.d_scan_all.ranges[i] > 0.0 and self.d_scan_all.ranges[i] < self.zone_circle_robot and fabs(x_point) > self.dx_robot :
                dem_circle_point_ahead_behind = dem_circle_point_ahead_behind + 1


        return dem_z1_t, dem_z2_t, dem_z3_t, dem_z1_s, dem_z2_s, dem_z3_s, dem_circle_point_beside, dem_circle_point_ahead_behind
                

    def run(self):
        while not rospy.is_shutdown() :
            np_ahead_z1 = np_ahead_z2 = np_ahead_z3 = 0
            np_behind_z1 = np_behind_z2 = np_behind_z3 = 0
            np_circle_point_beside = 0
            np_circle_point_ahead_behind = 0

            if self.is_scan_all == True:
                self.is_scan_all = False

                np_ahead_z1, np_ahead_z2, np_ahead_z3, np_behind_z1, np_behind_z2, np_behind_z3, np_circle_point_beside, np_circle_point_ahead_behind = self.check_zone_2head(self.d_scan_all.angle_min,\
                                            self.d_scan_all.angle_max,\
                                            self.dx1_head,self.dx2_head,self.dx3_head,\
                                            self.dx1_behind,self.dx2_behind,self.dx3_behind,\
                                            self.do_phan_giai_goc_quet)
                # check zone phia truoc
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
                
                # check zone phia sau
                if np_behind_z1 > 2:
                    np_behind_z1 = 0
                    self.zone_2head.zone_behind = 1

                elif np_behind_z2 > 1:
                    np_behind_z2 = 0
                    self.zone_2head.zone_behind = 2

                elif np_behind_z3 > 1:
                    np_behind_z3 = 0
                    self.zone_2head.zone_behind = 3

                else:
                    self.zone_2head.zone_behind = 0

                # check cricle 2 ben
                if np_circle_point_beside > 3:
                    self.zone_2head.zone_circle_beside = 1

                else:
                    self.zone_2head.zone_circle_beside = 0

                # check cricle truoc sau
                if np_circle_point_ahead_behind > 3:
                    self.zone_2head.zone_circle_ahead_behind = 1

                else:
                    self.zone_2head.zone_circle_ahead_behind = 0

                # print(self.zone_2head.zone_ahead)
                self.pub_zone.publish(self.zone_2head) 

            self.rate.sleep()
        # print('Thread #%s stopped' % self.threadID)

def main():
    try:
        m = safetyZone()
        m.run()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()
