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
from math import atan2, sin, cos, sqrt , fabs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import dynamic_reconfigure.client
import Queue
from sti_msgs.msg import Velocities ,Goal_control , Status_goalControl

#  pose Robot from Aruco
poseRbAr = PoseWithCovarianceStamped()  
enable_poseRbAr = False
#  pose Robot in Map
poseRbMa = Pose()


vel_robot = 0 
vel_0 = False
 
class quyTrinh(threading.Thread):

    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        rospy.init_node('goal_control', anonymous=True)
        self.rate = rospy.Rate(20)
        #giao tiep
        self.pub = rospy.Publisher('/status_goal_control', Status_goalControl, queue_size=10) 
        rospy.Subscriber("/goal_control", Goal_control, self.callSup_goal_control)

        rospy.Subscriber("/posePoseRobotFromAruco", PoseWithCovarianceStamped, self.callSup_posePose)
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
        #cmd_vel
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.on_shutdown(self.fnShutDown)

        #bien goal_control
        self.is_receivied_goal_control = False 
        self.goal_control = Goal_control()
        self.goal_control.send_goal = PoseStamped()
        self.goal_control.enable_move = 0
        self.goal_control.vel_x = 0.6
        self.goal_control.vel_theta = 0.5
        self.goal_control.tolerance_xy = 0.1
        self.goal_control.tolerance_theta = 0.1
        self.goal_control.planner_frequency = 5
        #bien goal status
        self.stt_gc = Status_goalControl()
        self.stt_gc.status_goal = 0
        self.stt_gc.check_received_goal = 0
        self.stt_gc.distance_goal = .0
        #dung tinh toan k/c
        self.poseRbMa = Pose()


    def pub_status(self, status_goal,check_received_goal, distance ):
        status = Status_goalControl()
        status.status_goal = status_goal
        status.check_received_goal = check_received_goal
        status.distance_goal = distance
        self.pub.publish(status)
    
    def callSup_goal_control(self,data):
        # if  self.is_receivied_goal_control == False :
        #     self.is_receivied_goal_control = True
        self.goal_control = data 
        
    def callSup_posePose(self, pose):
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
        self.poseRbMa = pose 
        
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
        
    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def constrain(self,val, min_val, max_val):
        if val < min_val: return min_val
        if val > max_val: return max_val
        return val

    def navi1_2(self,vel_x,vel_theta,tolerance_xy,tolerance_theta,planner_frequency):  
        # loc gia tri
        f_vel_x = self.constrain(vel_x,0.1,0.7)
        f_vel_theta = self.constrain(vel_theta,0.1,0.7)
        f_tolerance_xy = self.constrain(tolerance_xy,0.01,0.2)
        f_tolerance_theta = self.constrain(tolerance_theta,0.01,0.2)
        f_planner_frequency = self.constrain(planner_frequency,0,50)

        self.client1 = dynamic_reconfigure.client.Client("move_base", timeout=30)
        self.client1.update_configuration({"base_global_planner":"stiplanner/StiPlanner",\
                                            "base_local_planner" :"dwa_local_planner/DWAPlannerROS",\
                                            "planner_frequency":f_planner_frequency,\
                                            "recovery_behavior_enabled":"false",\
                                            "oscillation_distance":0,\
                                            "oscillation_timeout": 10,\
                                            "max_planning_retries":10})

        self.client2 = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS", timeout=30)
        self.client2.update_configuration({"xy_goal_tolerance":f_tolerance_xy,\
                                            "yaw_goal_tolerance" :f_tolerance_theta,\
                                            "max_vel_x":f_vel_x,\
                                            "max_vel_theta":f_vel_theta})

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist()) 

    def check_goal(sele,pose1, goal2, tolerance):
        if ( fabs(pose1.position.x) - fabs(goal2.target_pose.pose.position.x) ) < tolerance and \
           ( fabs(pose1.position.y) - fabs(goal2.target_pose.pose.position.y) ) < tolerance :
           return True 
        else : return False 

    def run(self):
        #goal2_vao
       
        print('Thread #%s started' % self.ident)
        buoc = 0
        while not self.shutdown_flag.is_set():

            #===== GOAL =====#
            # move with LIDAR
            if self.goal_control.enable_move == 1 :
                g_ctl_new = Goal_control()

                # nhan data
                if buoc == 0 :        
                    rospy.loginfo("buoc= %s",buoc)

                    # send check received goal
                    self.stt_gc.check_received_goal = 1 
                    rospy.loginfo("send check received goal")

                    # pub status
                    self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal,\
                                    self.stt_gc.distance_goal )
                    
                    # get new goal_control
                    ori = self.goal_control.send_goal.pose.orientation 
                    if ori.z == 99 and ori.w == 99 : # tinh ori 
                        buoc = 1 
                    else :
                        buoc = 2
                        #get Goal
                        g_ctl_new = self.goal_control 
                
                #quay goc
                if buoc == 1 :
                    # tinh goc robot hien tai
                    quata = ( self.poseRbMa.orientation.x,\
                            self.poseRbMa.orientation.y,\
                            self.poseRbMa.orientation.z,\
                            self.poseRbMa.orientation.w )
                    euler = euler_from_quaternion(quata)
                    theta_rb_ht = euler[2]
                    # theta_rb_ht = theta_rb_ht*180 / PI

                    # tinh goc poin to poin 
                    theta_poin = atan2(self.goal_control.send_goal.pose.position.y - self.poseRbMa.position.y, \
                                       self.goal_control.send_goal.pose.position.x - self.poseRbMa.position.x )
                    # theta_poin  = theta_poin*180 / PI 

                    # goc robot so voi target 
                    theta = theta_poin - theta_rb_ht
                    print "theta_rb_ht: %f - theta_poin: %f - theta: %f" %(theta_rb_ht,theta_poin,theta)

                    # quay
                    if fabs(theta) > 0.05 : # +- 10 do
                        if (theta > PI) or (theta > -PI and theta < 0 ): #quay phai , vel_z < 0
                            print "a"
                            twist = Twist()
                            twist.angular.z = -fabs(theta) - 0.02
                            if twist.angular.z < -0.5 : twist.angular.z = -0.5
                            self.pub_cmd_vel.publish(twist)
                            # buoc = 1

                        if (theta < -PI) or (theta < PI and theta > 0 ): #quay trai , vel_z > 0
                            print "b"
                            twist = Twist()
                            twist.angular.z = fabs(theta) + 0.02
                            if twist.angular.z > 0.5 : twist.angular.z = 0.5 
                            self.pub_cmd_vel.publish(twist)
                            # buoc = 1
                    else : 
                        self.pub_cmd_vel.publish(Twist())
                        g_ctl_new = self.goal_control
                        g_ctl_new.send_goal.pose.orientation = self.poseRbMa.orientation 
                        buoc = 2 
                
                # send goal
                if buoc == 2 :
                    
                    #clear costmap
                    self.clear_costmaps_srv(EmptyRequest())
                    rospy.loginfo("clear costmap")

                    #set param navi
                    self.navi1_2(g_ctl_new.vel_x, \
                                g_ctl_new.vel_theta, \
                                g_ctl_new.tolerance_xy, \
                                g_ctl_new.tolerance_theta, \
                                g_ctl_new.planner_frequency)
                    rospy.loginfo("set param navi")

                    # send goal
                    goal_new = MoveBaseGoal(g_ctl_new.send_goal)
                    self.send_goal(goal_new)
                    rospy.loginfo("send goal")

                    # pub status
                    self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal,\
                                    self.stt_gc.distance_goal )
                    buoc = 3
                    rospy.loginfo("pub status , buoc = 1")
                
                # get status
                if buoc == 3 :
                    rospy.loginfo("buoc= %s",buoc)
                    self.stt_gc.status_goal = self.client.get_state()
                    self.stt_gc.distance_goal = self.fnCalcDistPoints(  self.poseRbMa.position.x,\
                                                                        goal_new.target_pose.pose.position.x,\
                                                                        self.poseRbMa.position.y,\
                                                                        goal_new.target_pose.pose.position.y )
                    if self.stt_gc.status_goal == 3 :
                        if self.check_goal(self.poseRbMa,goal_new,self.goal_control.tolerance_xy) == False :
                        # if self.stt_gc.distance_goal > self.goal_control.tolerance_xy :
                            self.stt_gc.status_goal = 10 # error

                    self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal,\
                                        self.stt_gc.distance_goal )
                    rospy.loginfo("status_goal= %s - distance_goal= %s",self.stt_gc.status_goal,self.stt_gc.distance_goal) 

            elif self.goal_control.enable_move == -1 :
                self.stt_gc.check_received_goal = -1
                self.stt_gc.status_goal = -1
                self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal,\
                                self.stt_gc.distance_goal )
                self.client.cancel_all_goals()
                rospy.loginfo("cancel_all_goals")
                buoc = 0

            elif self.goal_control.enable_move == 0 :
                rospy.loginfo("reset and wait command")
                self.stt_gc.check_received_goal = -1
                self.stt_gc.status_goal = -1
                self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal,\
                                self.stt_gc.distance_goal )
                buoc = 0

            self.rate.sleep()

        print('Thread 1 #%s stopped' % self.ident)

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
        thread1 = quyTrinh(1)
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
