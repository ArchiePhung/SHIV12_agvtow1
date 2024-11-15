#!/usr/bin/python

import threading
import time
import rospy
from std_msgs.msg import String

import sys
import struct
import string
import roslib
import serial
import signal
# clear costmap
from std_srvs.srv import Empty, EmptyRequest

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import *
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose, Twist
from sensor_msgs.msg import LaserScan

from math import pi as PI
from math import atan2, sin, cos, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import dynamic_reconfigure.client
import Queue
from sti_msgs.msg import Velocities

#  pose Robot from Aruco
poseRbAr = PoseWithCovarianceStamped()  
enable_poseRbAr = False
#  pose Robot in Map
poseRbMa = Pose()
enable_poseRbMa = False

vel_robot = 0 
vel_0 = False
 
class quyTrinh(threading.Thread):

    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

        rospy.init_node('quytrinh', anonymous=True)
        self.pub = rospy.Publisher('/parking_server', Pose, queue_size=10) 
        rospy.Subscriber("/parking_client", Pose, self.callSup)
        self.rate = rospy.Rate(1) # 1hz
        rospy.Subscriber("/posePoseRobotFromAruco", PoseWithCovarianceStamped, self.callSup1)
        rospy.Subscriber("/raw_vel", Velocities, self.callSup2)
        rospy.Subscriber("/robot_pose", Pose, self.callSup3)

        #cancel Goal
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        #clear_costmap
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        #set pose
        self.initpose_publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 100)
        #ReserMap hector
        self.ReserMap = rospy.Publisher('syscommand', String, queue_size = 100)

        # The shutdown_flag is a threading.Event object that
        # indicates whether the thread should be terminated.
        self.shutdown_flag = threading.Event()

        #bien
        self.idtag_parking = 0
        self.park_info = 0

    def enable_park(self, id_tag):
        park = Pose()
        park.position.x = id_tag
        self.pub.publish(park)
    
    def callSup(self,data):
        self.park_info = data.position.x

    def callSup1(self, pose):
        global enable_poseRbAr
        global poseRbAr
        poseRbAr = pose
        enable_poseRbAr = True

    def locPoseRb(self, i):
        global enable_poseRbAr , poseRbAr
        poseLoc = PoseWithCovarianceStamped() 
        dem = 0 
        for dem in range(i):
            while (enable_poseRbAr == False): 
                dem = dem
            else:     
                poseLoc.pose.pose.position.x    = poseLoc.pose.pose.position.x    + poseRbAr.pose.pose.position.x    
                poseLoc.pose.pose.position.y    = poseLoc.pose.pose.position.y    + poseRbAr.pose.pose.position.y    
                poseLoc.pose.pose.orientation.z = poseLoc.pose.pose.orientation.z + poseRbAr.pose.pose.orientation.z 
                poseLoc.pose.pose.orientation.w = poseLoc.pose.pose.orientation.w + poseRbAr.pose.pose.orientation.w 

                dem = dem + 1
                enable_poseRbAr = False
                # print "dem= " , dem 
                # print "x= " ,poseRbAr.pose.pose.position.x   
                # print "y= " ,poseRbAr.pose.pose.position.y   
                # print "z= " ,poseRbAr.pose.pose.orientation.z
                # print "w= " ,poseRbAr.pose.pose.orientation.w
                           
        poseLoc.pose.pose.position.x    = poseLoc.pose.pose.position.x    / i
        poseLoc.pose.pose.position.y    = poseLoc.pose.pose.position.y    / i
        poseLoc.pose.pose.orientation.z = poseLoc.pose.pose.orientation.z / i
        poseLoc.pose.pose.orientation.w = poseLoc.pose.pose.orientation.w / i

        return poseLoc

    def callSup2(self, vel):
        global vel_0 
        if ( vel.angular_z == 0 and vel.linear_x ==0 ): vel_0 = True
        else : vel_0 = False

    def callSup3(self, pose):
        global poseRbMa
        poseRbMa = pose 
        enable_poseRbMa = True
        # print "\nx= " ,poseRbMa.position.x   
        # print "y= " ,poseRbMa.position.y   
        # print "z= " ,poseRbMa.orientation.z
        # print "w= " ,poseRbMa.orientation.w
        
    def send_goal(self, g):
        goal = MoveBaseGoal()
        goal = g
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(goal)
 
    def SendInitialPose(self,InitialPose, initial_pose, time_stamp):
        # goal: [x, y, yaw]
        InitialPoseMsg = PoseWithCovarianceStamped()
        InitialPoseMsg = initial_pose
        InitialPoseMsg.header.stamp = time_stamp
        InitialPoseMsg.header.frame_id = 'map'

        InitialPose.publish(InitialPoseMsg)

    def navi1_1(self):
        self.client1 = dynamic_reconfigure.client.Client("move_base", timeout=30)
        self.client1.update_configuration({"base_global_planner":"stiplanner/StiPlanner",\
                                            "base_local_planner" :"sti_localplanner/StiLocalPlanner",\
                                            "planner_frequency":0,\
                                            "recovery_behavior_enabled":"false",\
                                            "oscillation_distance":10,\
                                            "oscillation_timeout": 10,\
                                            "max_planning_retries":10})

        self.client2 = dynamic_reconfigure.client.Client("move_base/StiLocalPlanner", timeout=30)
        self.client2.update_configuration({"tolerance_trans":0.03,\
                                            "tolerance_rot" :0.1,\
                                            "max_vel_lin":0.2,\
                                            "max_vel_th":0.3})
    
    def navi1_2(self):  
        self.client1 = dynamic_reconfigure.client.Client("move_base", timeout=30)
        self.client1.update_configuration({"base_global_planner":"stiplanner/StiPlanner",\
                                            "base_local_planner" :"dwa_local_planner/DWAPlannerROS",\
                                            "planner_frequency":5,\
                                            "recovery_behavior_enabled":"false",\
                                            "oscillation_distance":0,\
                                            "oscillation_timeout": 10,\
                                            "max_planning_retries":10})

        self.client2 = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS", timeout=30)
        self.client2.update_configuration({"xy_goal_tolerance":0.1,\
                                            "yaw_goal_tolerance" :0.1,\
                                            "max_vel_x":0.6,\
                                            "max_vel_theta":0.5})

    def run(self):
        print('Thread #%s started' % self.ident)
        goal2_vao = MoveBaseGoal()
        goal2_bt = MoveBaseGoal()
        goal2_q = MoveBaseGoal()
        goal4_vao = MoveBaseGoal()
        goal4_bt = MoveBaseGoal()
        goal4_q = MoveBaseGoal()
        p1_th = MoveBaseGoal()
        p1_ng = MoveBaseGoal()
        p2_th = MoveBaseGoal()
        p2_ng = MoveBaseGoal()
        p3_th = MoveBaseGoal()

        dem = int()
        global enable_poseRbAr , poseRbAr , vel_0 

       #goal2_vao
        goal2_vao.target_pose.header.frame_id    = "map"
        goal2_vao.target_pose.pose.position.x    = 0.166
        goal2_vao.target_pose.pose.position.y    = -0.083
        goal2_vao.target_pose.pose.orientation.z = 0.007
        goal2_vao.target_pose.pose.orientation.w = 1.000
       #goal2_bt
        goal2_bt.target_pose.header.frame_id    = "map"
        goal2_bt.target_pose.pose.position.x    = 0.126
        goal2_bt.target_pose.pose.position.y    = -0.386
        goal2_bt.target_pose.pose.orientation.z = 0.958
        goal2_bt.target_pose.pose.orientation.w = 0.287
       #goal2_q
        goal2_q.target_pose.header.frame_id    = "map"
        goal2_q.target_pose.pose.position.x    = 0.178
        goal2_q.target_pose.pose.position.y    = -0.111
        goal2_q.target_pose.pose.orientation.z = 1.000
        goal2_q.target_pose.pose.orientation.w = -0.022

       #goal4_vao

        goal4_vao.target_pose.header.frame_id    = "map"
        goal4_vao.target_pose.pose.position.x    = 10.238
        goal4_vao.target_pose.pose.position.y    = -7.897
        goal4_vao.target_pose.pose.orientation.z = -0.862
        goal4_vao.target_pose.pose.orientation.w = 0.506
       #goal4_bt
        goal4_bt.target_pose.header.frame_id    = "map"
        goal4_bt.target_pose.pose.position.x    = 9.888
        goal4_bt.target_pose.pose.position.y    = -8.511
        goal4_bt.target_pose.pose.orientation.z = 0.873
        goal4_bt.target_pose.pose.orientation.w = -0.487
       #goal4_q
        goal4_q.target_pose.header.frame_id    = "map"
        goal4_q.target_pose.pose.position.x    = 10.238
        goal4_q.target_pose.pose.position.y    = -7.897
        goal4_q.target_pose.pose.orientation.z = 0.454
        goal4_q.target_pose.pose.orientation.w = 0.891

       #p1_th
        p1_th.target_pose.header.frame_id    = "map"
        p1_th.target_pose.pose.position.x    = -0.774
        p1_th.target_pose.pose.position.y    = -0.654
        p1_th.target_pose.pose.orientation.z = 1.000
        p1_th.target_pose.pose.orientation.w = 0.026
       #p1_ng
        p1_ng.target_pose.header.frame_id    = "map"
        p1_ng.target_pose.pose.position.x    = -0.790
        p1_ng.target_pose.pose.position.y    = -0.670
        p1_ng.target_pose.pose.orientation.z = 0.223
        p1_ng.target_pose.pose.orientation.w = 0.975
       #p2_th
        p2_th.target_pose.header.frame_id    = "map"
        p2_th.target_pose.pose.position.x    = 11.476
        p2_th.target_pose.pose.position.y    = -5.915
        p2_th.target_pose.pose.orientation.z = -0.857
        p2_th.target_pose.pose.orientation.w = 0.516
       #p2_ng
        p2_ng.target_pose.header.frame_id    = "map"
        p2_ng.target_pose.pose.position.x    = 11.673
        p2_ng.target_pose.pose.position.y    = -5.984
        p2_ng.target_pose.pose.orientation.z = 0.964
        p1_ng.target_pose.pose.orientation.w = 0.265

       #
        dem = -2
        poseSet = PoseWithCovarianceStamped() 
        self.navi1_2()
        while not self.shutdown_flag.is_set():
            print "quytrinh_test= ", dem
            if dem == -2 :
                self.enable_park(11)
                print self.park_info
                if self.park_info != -1 :
                    dem = -1

            print "quytrinh_test= ", dem
            if dem == -1 :
                self.enable_park(0)
                if self.park_info == 0 :
                    dem = 0 
            
            print "quytrinh= ", dem
            if dem == 1 :
                self.navi1_2()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                # time.sleep(0.5)               
                self.send_goal(goal2_vao)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED : dem = 20
                else: 
                    dem = 1
                    print " error 1 "
            
            print "quytrinh= ", dem
            if dem == 20 :
                while(vel_0 != True):
                    dem = 20
                    print " Doi vel = 0 ..."
                dem = 2

            print "quytrinh= ", dem
            if dem == 2 :
                self.navi1_1()
                
                # while(vel_0 != True):
                #     dem = 2 
                #     print " Doi vel = 0 ..."
                # if vel_0 == True :

                print " vel = 0 ..."
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"

                for i in xrange(5):
                    self.ReserMap.publish("reset")
                    rospy.sleep(0.1)

                # while(enable_poseRbAr != True): print " Doi enable_poseRbAr... "
                poseSet= self.locPoseRb(10)

                self.SendInitialPose(self.initpose_publisher,poseSet,rospy.Time.now())
                time.sleep(0.5)
                print "Set pose OK "

                self.send_goal(goal2_bt)
                
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED :
                    time.sleep(5)
                    dem = 3
                else: 
                    dem = 2
                    print " error 2 "

            print "quytrinh= ", dem
            if dem == 3 :
                self.navi1_2()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(goal2_q)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED : dem = 4
                else: 
                    dem = 3
                    print " error 3 "

            print "quytrinh= ", dem
            if dem == 4 :
                self.navi1_2()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(p1_th)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED : dem = 5
                else: 
                    dem = 4
                    print " error 4 "

            print "quytrinh= ", dem
            if dem == 5 :
                self.navi1_2()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(p2_th)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED : dem = 6
                else: 
                    dem = 5
                    print " error 5 "
            
            print "quytrinh= ", dem
            if dem == 6 :
                self.navi1_2()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(goal4_vao)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED : dem = 70
                else: 
                    dem = 6
                    print " error 7 "
            
            print "quytrinh= ", dem
            if dem == 70 :
                while(vel_0 != True):
                    dem = 70
                    print " Doi vel = 0 ..."
                dem = 7

            print "quytrinh= ", dem
            if dem == 7 :
                self.navi1_1()
                # while(enable_poseRbAr != True): print " Doi enable_poseRbAr... "
                # while(vel_0 != True):
                #     dem =7
                #     print " Doi vel = 0 ..."
                # if vel_0 == True :
                print " vel = 0 ..."
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"

                for i in xrange(5):
                    self.ReserMap.publish("reset")
                    rospy.sleep(0.1)
                poseSet= self.locPoseRb(10)

                self.SendInitialPose(self.initpose_publisher,poseSet,rospy.Time.now())
                time.sleep(0.5)
                print "Set pose OK "

                self.send_goal(goal4_bt)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED :
                    time.sleep(5)
                    dem = 8
                else: 
                    dem = 7
                    print " error 7 "
            
            print "quytrinh= ", dem
            if dem == 8 :
                self.navi1_2()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"

                self.send_goal(goal4_q)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED : dem = 9
                else: 
                    dem = 8
                    print " error 8 "

            print "quytrinh= ", dem
            if dem == 9 :
                self.navi1_2()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(p2_ng)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED : dem = 10
                else: 
                    dem = 9
                    print " error 9 "

            print "quytrinh= ", dem
            if dem == 10 :
                self.navi1_2()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(p1_ng)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED : dem = 1
                else: 
                    dem = 10
                    print " error 10 "
        
            time.sleep(0.01)

        print('Thread #%s stopped' % self.ident)

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
 
    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:
        thread1 = quyTrinh(1,"quyTrinh")
        thread1.start()
 
        # Keep the main thread running, otherwise signals are ignored.
        while True:
            time.sleep(0.01)
 
    except ServiceExit:
        # Terminate the running threads.
        # Set the shutdown flag on each thread to trigger a clean shutdown of each thread.
        thread1.shutdown_flag.set()
        # Wait for the threads to close...
        thread1.join()
    print('Exiting main program')
 
 
if __name__ == '__main__':
    main()