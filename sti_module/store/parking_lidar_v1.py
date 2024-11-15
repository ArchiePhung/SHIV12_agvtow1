#!/usr/bin/env python
#Authors : Anh Tuan 
#Start : 11/5/2020 -> ...

import rospy
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point , Pose
from std_msgs.msg import Empty , ColorRGBA
from visualization_msgs.msg import Marker , MarkerArray
import numpy as nu
from math import sin , cos , pi , atan2 ,sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import threading
import signal
from pykalman import KalmanFilter

class Parking_lidar(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        self.cmd_pub = rospy.Publisher('cmd_vel',Twist , queue_size=1)
        self.reset_pub = rospy.Publisher('reset', Empty , queue_size=1)
        self.laser_pub = rospy.Publisher('scan_park', LaserScan, queue_size=1) # pub data guong
        # self.marker_pub = rospy.Publisher('marker_guong', MarkerArray, queue_size=1)
        self.marker_pub = rospy.Publisher('marker_guong', Marker, queue_size=1)
        rospy.Subscriber("scan_tim551", LaserScan, self.call_sup1)
        rospy.Subscriber("odom",Odometry,self.call_sup2)

        self.msg_scan = LaserScan()
        self.is_scan_received = False
        self.msg_odom = Odometry()
        self.is_odom_recived = False

        self.filter_scan = LaserScan()
        self.muc_intensities = 240
        #tong : 231  point , giua : 115
        self.center_p_tim551 = 115 -1
        self.start_point = 0
        self.end_point = 0
        self.center_point = .0
        self.angle_increment = 0.0174532923847 #rad tim551 :1 do, lms100 : 0.25 do
        self.angle_min = .0
        self.angle_max = .0

    def call_sup1(self,data): #15Hz
        self.msg_scan =  data
        if self.is_scan_received == False : self.is_scan_received = True

    def call_sup2(self,data):
        self.msg_odom = data
        if self.is_odom_recived == False : self.is_odom_recived = True

    def fnCalcDistPoints(self, x1, x2, y1, y2):
        # print "fnCalcDistPoints"
        return sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def filter_guong(self,msg_scan):
       # LaserScan 
        # std_msgs/Header header
        # uint32 seq
        # time stamp
        # string frame_id
        # float32 angle_min
        # float32 angle_max
        # float32 angle_increment
        # float32 time_increment
        # float32 scan_time
        # float32 range_min
        # float32 range_max
        # float32[] ranges
        # float32[] intensities

        # frame_id: "laser_tim551"
        # angle_min: -2.00712871552 <=> 115 do
        # angle_max: 2.00712871552
        # angle_increment: 0.0174532923847
        # time_increment: 0.000185185184819
        # scan_time: 0.0666666701436
        # range_min: 0.0500000007451
        # range_max: 10.0
       #    
        sl_point    = 0 
        start_point = 0
        end_point   = 0
        center_point = .0 
        if self.is_scan_received == True:
            self.is_scan_received = False
            # print "tong_point: ", len(msg_scan.intensities)
            self.filter_scan = msg_scan
            self.angle_min = msg_scan.angle_min
            self.angle_max = msg_scan.angle_max
            self.angle_increment = msg_scan.angle_increment
            list_ranges = list(msg_scan.ranges)
            list_intensities = list(msg_scan.ranges)
            for i in range(len(msg_scan.ranges)):
                # print i # 0-230

                if msg_scan.intensities[i] > self.muc_intensities :
                    end_point = i
                    sl_point = sl_point +1
                    # print "i: %s - ranges: %s - intensities: %s" % (i,msg_scan.ranges[i],msg_scan.intensities[i])
                    list_ranges[i] = msg_scan.ranges[i]
                    list_intensities[i] = msg_scan.intensities[i]
                    
                else :
                    list_ranges[i] = 0
                    list_intensities[i] = 0

           #tinh toan

            start_point = end_point - sl_point 
            center_point = end_point - sl_point / 2.
            print "start_point: %s , center_point: %s , end_point: %s " % (start_point,center_point,end_point)

            # center_p_tim551    <->  0 rad
            # center_p_tim551 +1 <->  angle_increment rad

            goc_sp = (start_point  - self.center_p_tim551 ) * msg_scan.angle_increment
            goc_ep = (end_point    - self.center_p_tim551 ) * msg_scan.angle_increment
            goc_cp = (center_point - self.center_p_tim551 ) * msg_scan.angle_increment
            print "goc_sp= %s, goc_ep= %s, goc_cp= %s " %(goc_sp,goc_ep,goc_cp)
            
            ds = msg_scan.ranges[start_point+1] 
            de = msg_scan.ranges[end_point-1] 
            dc = msg_scan.ranges[int(center_point)] 
            print "ds= %s, de= %s, dc= %s " %(ds,de,dc)

            x_sp = ds * cos(goc_sp)
            y_sp = ds * sin(goc_sp)
            x_ep = de * cos(goc_ep)
            y_ep = de * sin(goc_ep)
            x_cp = dc * cos(goc_cp)
            y_cp = dc * sin(goc_cp)
            print "x_sp: %s , y_sp: %s " % (x_sp,y_sp)       
            print "x_ep: %s , y_ep: %s " % (x_ep,y_ep)  
            print "x_cp: %s , y_cp: %s " % (x_cp,y_cp)      

            dodai_guong = self.fnCalcDistPoints(x_sp,x_ep,y_sp,y_ep)
            print "dodai_guong= " , dodai_guong

           # pub marker 
            line_color = ColorRGBA()       
            line_color.r = 0
            line_color.g = 1
            line_color.b = 0
            line_color.a = 1.0
            start_point = Point()        #start point
            start_point.x = x_sp
            start_point.y = y_sp
            start_point.z = 0
            end_point = Point()        #end point
            end_point.x = x_ep
            end_point.y = y_ep
            end_point.z = 0
            cen_point = Pose()
            # cen_point.position.x = x_cp
            # cen_point.position.y = y_cp
            cen_point.orientation.w = 1.0

            msg_marker = Marker()
            msg_marker.id = 3
            msg_marker.header.frame_id = 'laser_tim551'
            msg_marker.header.stamp = rospy.Time()
            msg_marker.type = Marker.LINE_STRIP
            msg_marker.ns = 'guong'
            msg_marker.action = 0
            msg_marker.scale.x = 0.05
            msg_marker.pose = cen_point
            # msg_marker.text = "reflectors"
            msg_marker.points.append(start_point)
            msg_marker.points.append(end_point)
            msg_marker.colors.append(line_color)
            msg_marker.colors.append(line_color)

            self.marker_pub.publish( msg_marker )

           # pub /scan_filter
            self.filter_scan.ranges      = tuple(list_ranges)
            self.filter_scan.intensities = tuple(list_intensities)
            self.laser_pub.publish(self.filter_scan)
            
            return start_point , center_point , end_point

    def run(self):
        dem = 1
        while not self.shutdown_flag.is_set():
            # print "run"
            if dem == 1 : self.filter_guong(self.msg_scan)
                # if self.is_scan_received == True:
                #     self.start_point, self.center_point ,self.end_point = self.filter_guong(self.msg_scan)
                #     print "start_point: %s , center_point: %s , end_point: %s " % (self.start_point,\
                #                                                                    self.center_point,\
                #                                                                    self.end_point)

                    

            time.sleep(0.01)
        print('Thread #%s stopped' % self.threadID)


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
    rospy.init_node('parking_lidar')
    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:
        thread1 = Parking_lidar(1)
        thread1.start()
 
        # Keep the main thread running, otherwise signals are ignored.
        while True:
            time.sleep(0.01)
 
    except ServiceExit:
        thread1.shutdown_flag.set()
        thread1.join()
    print('Exiting main program')
 
if __name__ == '__main__':
    main()