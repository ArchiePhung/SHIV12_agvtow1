#!/usr/bin/env python
#Author AnhTuan
#Start 22/5/2020

from leg_tracker.msg import LegArray
from visualization_msgs.msg import Marker , MarkerArray
from std_msgs.msg import Empty , ColorRGBA
from geometry_msgs.msg import  Pose , Point
from sti_msgs.msg import *
from math import atan2, sin, cos, sqrt, fabs
import rospy
import time
import threading
import signal

class Leg_detection(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        self.rate = rospy.Rate(15)
        rospy.Subscriber("/detected_leg_clusters",LegArray, self.call_sub1)
        rospy.Subscriber("/zone_3d",Safety_zone, self.call_sub2)
        self.marker_pub = rospy.Publisher('marker_guong', Marker, queue_size=1)
        self.zone_lidar_pub = rospy.Publisher('zone_lidar',Safety_zone, queue_size=10)
        self.zone_robot_pub = rospy.Publisher('zone_robot',Safety_zone, queue_size=10)
        self.legData = LegArray()
        self.is_detec_leg = False

        #get param 
        self.dx_robot = rospy.get_param('~dx_robot',0.32)
        self.dy_robot = rospy.get_param('~dy_robot',0.26)
        self.dx_lidar = rospy.get_param('~dx_lidar',0.32)
        self.dy_lidar = rospy.get_param('~dy_lidar',0.26)
        self.dx1 = rospy.get_param('~dx1',1)
        self.dx2 = rospy.get_param('~dx2',2)
        self.dx3 = rospy.get_param('~dx3',3)

        self.zone_lidar = Safety_zone()
        self.zone_3d = Safety_zone()
        self.zone_robot = Safety_zone()

    def call_sub1(self,data):
        self.legData = data
        if self.is_detec_leg == False : self.is_detec_leg = True
        # print "diem: " ,len(self.legData.legs)
        # print self.legData.legs[1]

    def call_sub2(self,data):
        self.zone_3d = data

    def check_zone(self,p,dx,dy):
        if (fabs(p.x) < dx) and (fabs(p.y)<dy) : 
            return True 
        else:
            return False


    def run(self):
        while not self.shutdown_flag.is_set():
            if self.is_detec_leg == True :
                self.is_detec_leg = False
                dem_z1 = dem_z2 = dem_z3 = 0
                dolech_lidar = self.dx_robot 
                for i in range(len(self.legData.legs)):
                    # print "sodiem: " ,len(self.legData.legs)
                    # print "i: ",i
                    #zon1 1
                    zone1 = self.check_zone(self.legData.legs[i].position, self.dx1 + dolech_lidar, self.dx_robot)
                    if zone1 == True : dem_z1 = dem_z1 + 1
                    #zone 2
                    zone2 = self.check_zone(self.legData.legs[i].position, self.dx2 + dolech_lidar, self.dx_robot)
                    if zone2 == True : dem_z2 = dem_z2 + 1
                    #zone 3
                    zone3 = self.check_zone(self.legData.legs[i].position, self.dx3 + dolech_lidar, self.dx_robot)
                    if zone3 == True : dem_z3 = dem_z3 + 1

                if dem_z1 != 0 : 
                    dem_z1 = 0
                    self.zone_lidar.zone1 = True
                else : self.zone_lidar.zone1 = False

                if dem_z2 != 0 : 
                    dem_z2 = 0
                    self.zone_lidar.zone2 = True
                else : self.zone_lidar.zone2 = False

                if dem_z3 != 0 : 
                    dem_z3 = 0
                    self.zone_lidar.zone3 = True
                else : self.zone_lidar.zone3 = False

                print ("sodiem: " ,len(self.legData.legs))
                print ("zone: ")
                print (self.zone_lidar)

                self.zone_lidar_pub.publish(self.zone_lidar)
                
              # msg_marker = Marker()
                # # pub marker 
                # line_color = ColorRGBA()       
                # line_color.r = 0
                # line_color.g = 1
                # line_color.b = 0
                # line_color.a = 1.0

                # # cen_point = Pose()
                # # cen_point.position.x = self.legData.legs[i].position.x
                # # cen_point.position.y = self.legData.legs[i].position.y
                # # cen_point.orientation.w = 1.0

                # msg_marker.id = 0
                # msg_marker.header.frame_id = 'base_footprint'
                # msg_marker.header.stamp = rospy.Time()
                # msg_marker.ns = "leg";
                # msg_marker.id = 0
                # msg_marker.type = Marker.POINTS
                # msg_marker.action = Marker.ADD

                # msg_marker.scale.x = 0.1
                # msg_marker.scale.y = 0.1
                # msg_marker.scale.z = 0.1
                # # msg_marker.pose.position.x = self.legData.legs[i].position.x
                # # msg_marker.pose.position.y = self.legData.legs[i].position.y
                
                # msg_marker.color.r = 0.0
                # msg_marker.color.g = 1.0
                # msg_marker.color.b = 0.0
                # msg_marker.color.a = 1.0

                # for i in range(len(self.legData.legs)):
                #     print "sodiem: " ,len(self.legData.legs)
                #     print "i: ",i
                #     msg_marker.points.append(self.legData.legs[i].position)

                    

                # self.marker_pub.publish( msg_marker )


            # ket hop zone 2d , 3d 
            if (self.zone_lidar.zone1 == 0 and self.zone_3d.zone1 == 0): 
                self.zone_robot.zone1 = 0
            else : self.zone_robot.zone1 = 1

            if (self.zone_lidar.zone2 == 0 and self.zone_3d.zone2 == 0): 
                self.zone_robot.zone2 = 0
            else : self.zone_robot.zone2 = 1

            if (self.zone_lidar.zone3 == 0 and self.zone_3d.zone3 == 0): 
                self.zone_robot.zone3 = 0
            else : self.zone_robot.zone3 = 1

            self.zone_robot_pub.publish(self.zone_robot)


            self.rate.sleep()
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
    rospy.init_node('safety_zone')
    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:
        thread1 = Leg_detection(1)
        thread1.start()
        # thread2 = Connect_ros(2)
        # thread2.start()
 
        # Keep the main thread running, otherwise signals are ignored.
        while True:
            time.sleep(0.01)
 
    except ServiceExit:
        thread1.shutdown_flag.set()
        thread1.join()
        # thread2.shutdown_flag.set()
        # thread3.join()
    print('Exiting main program')
 
if __name__ == '__main__':
    main()