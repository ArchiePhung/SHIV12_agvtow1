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
        self.zone_robot_pub = rospy.Publisher('zone_robot',Zone_lidar_2head, queue_size=10)
        self.zone_lidar_2head_pub = rospy.Publisher('zone_lidar_2head',Zone_lidar_2head, queue_size=10)
        self.legData = LegArray()
        self.is_detec_leg = False

        #get param 
        self.dx_robot = rospy.get_param('~dx_robot',0.32)
        self.dy_robot = rospy.get_param('~dy_robot',0.26)
        self.dx1 = rospy.get_param('~dx1',1)
        self.dx2 = rospy.get_param('~dx2',2)
        self.dx3 = rospy.get_param('~dx3',3)

        self.zone_lidar = Safety_zone()
        self.zone_3d = Safety_zone()
        self.zone_robot = Zone_lidar_2head()
        self.zone_2head = Zone_lidar_2head()


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

    def check_zone_2head(self, p, dx, dy):
        if p.x > 0 : # ahead 
            if (p.x < dx) and (p.x > self.dx_robot) and (fabs(p.y)<dy) : 
                return 1 
            else:
                return 0
        else: # behind
            if (p.x > -dx) and (p.x < -self.dx_robot) and  (fabs(p.y)<dy) : 
                return 2 
            else:
                return 0

    def run(self):
        while not self.shutdown_flag.is_set():
            if self.is_detec_leg == True :
                self.is_detec_leg = False
                dem_z1_t = dem_z2_t = dem_z3_t = 0
                dem_z1_s = dem_z2_s = dem_z3_s = 0
    
                for i in range(len(self.legData.legs)):
                    #zon1 1
                    zone1 = self.check_zone_2head(self.legData.legs[i].position, self.dx1 + self.dx_robot, self.dy_robot)
                    # if zone1 == 0 : dem_z1_t = dem_z1_s = 0 
                    if zone1 == 1 : dem_z1_t = dem_z1_t + 1
                    if zone1 == 2 : dem_z1_s = dem_z1_s + 1

                    # zone 2
                    zone2 = self.check_zone_2head(self.legData.legs[i].position, self.dx2 + self.dx_robot, self.dy_robot)
                    if zone2 == 1 : dem_z2_t = dem_z2_t + 1
                    if zone2 == 2 : dem_z2_s = dem_z2_s + 1

                    # zone 3
                    zone3 = self.check_zone_2head(self.legData.legs[i].position, self.dx3 + self.dx_robot, self.dy_robot)
                    if zone3 == 1 : dem_z3_t = dem_z3_t + 1
                    if zone3 == 2 : dem_z3_s = dem_z3_s + 1                    
                
                # zone ahead
                if dem_z1_t != 0 : 
                    dem_z1_t = 0
                    self.zone_2head.zone_ahead = 1

                elif dem_z2_t != 0 : 
                    dem_z2_t = 0
                    self.zone_2head.zone_ahead = 2

                elif dem_z3_t != 0 : 
                    dem_z3_t = 0
                    self.zone_2head.zone_ahead = 3
                else :
                    self.zone_2head.zone_ahead = 0

              # zone behind
                if dem_z1_s != 0 : 
                    dem_z1_s = 0
                    self.zone_2head.zone_behind = 1
                    # print ("A")

                elif dem_z2_s != 0 : 
                    dem_z2_s = 0
                    self.zone_2head.zone_behind = 2
                    # print ("    B")

                elif dem_z3_s != 0 : 
                    dem_z3_s = 0
                    self.zone_2head.zone_behind = 3
                    # print ("        C")
                else :
                    self.zone_2head.zone_behind = 0

                # print ("sodiem: " ,len(self.legData.legs))
                # print ("zone: ")
                # print (self.zone_2head)

                self.zone_lidar_2head_pub.publish(self.zone_2head)
                
            
            # ket hop zone 2d , 3d 
            if (self.zone_2head.zone_ahead == 1 or self.zone_3d.zone1 == 1): 
                self.zone_robot.zone_ahead = 1

            elif (self.zone_2head.zone_ahead == 2 or self.zone_3d.zone2 == 1): 
                self.zone_robot.zone_ahead = 2

            elif (self.zone_2head.zone_ahead == 3 or self.zone_3d.zone3 == 1): 
                self.zone_robot.zone_ahead = 3

            else: 
                self.zone_robot.zone_ahead = 0

            self.zone_robot.zone_behind = self.zone_2head.zone_behind
            self.zone_robot_pub.publish(self.zone_robot)

            # print (self.zone_robot)


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
    rospy.init_node('safety_zone_v2')
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