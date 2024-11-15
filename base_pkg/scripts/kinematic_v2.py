#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#Authors : BEE
# kinematic.
# DATE: 20/10/2021
# AUTHOR: HOANG VAN QUANG
# UPDATE: PHUCHOANG bo sung odometry

import rospy
import sys
import time
import threading
import signal

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped, Pose, Quaternion
from message_pkg.msg import Driver_query
from message_pkg.msg import Driver_respond

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from sti_msgs.msg import Velocities
from std_msgs.msg import Int16

from math import sin , cos , pi , atan2, radians, degrees

# import rospy
# import sys
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Empty
# import numpy as nu
# from math import sin , cos , pi , atan2
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import time
# import threading
# import signal
# from pykalman import KalmanFilter
"""
Lỗi phiên bản 1:
	Dừng code khi thực hiện lệnh ghi tốc độ.
	Có trường hợp Driver báo lỗi rồi nhưng vẫn quay.
"""
class RPM:
	motor1 = 0
	motor2 = 0

class kinematic(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()

        print("ROS Initial!")
        rospy.init_node('kinematic', anonymous= False) # False
        self.rate = rospy.Rate(100)

        # -- parameter
        self.wheel_circumference = rospy.get_param("wheel_circumference", 0.47)
        self.transmission_ratio = rospy.get_param("transmission_ratio", 30.0)
        self.distanceBetwentWheels = rospy.get_param("distanceBetwentWheels", 0.541)
        self.frequency_control = rospy.get_param("frequency_control", 25.0)
        self.linear_max = rospy.get_param("linear_max", 0.8) # m/s
        self.angular_max = rospy.get_param("angular_max", 0.5) # rad/s
        self.max_rpm = rospy.get_param("max_rpm", 3800)

        self.topicControl_vel =  rospy.get_param("topicControl_vel", "/cmd_vel") # 
        self.topicGet_vel =  rospy.get_param("topicGet_vel", "/raw_vel") # 

        self.topicControl_driverLeft = rospy.get_param("topicControl_driverLeft", "/driver1_query")
        self.topicControl_driverRight = rospy.get_param("topicControl_driverRight", "/driver2_query")
        self.topicRespond_driverLeft = rospy.get_param("topicRespond_driverLeft", "/driver1_respond")
        self.topicRespond_driverRight = rospy.get_param("topicRespond_driverRight", "/driver2_respond")

        self.frame_id = rospy.get_param("frame_id", "frame_robot")
        # -----------------
        rospy.Subscriber(self.topicControl_vel, Twist, self.cmdVel_callback)
        self.cmd_vel = Twist()

        rospy.Subscriber("/task_driver", Int16, self.taskDriver_callback)
        self.task_driver = Int16()

        rospy.Subscriber(self.topicRespond_driverLeft, Driver_respond, self.driver1Respond_callback)
        self.driver1_respond = Driver_respond()

        rospy.Subscriber(self.topicRespond_driverRight, Driver_respond, self.driver2Respond_callback)
        self.driver2_respond = Driver_respond()

        # -----------------
        self.pub_rawVel = rospy.Publisher(self.topicGet_vel, TwistWithCovarianceStamped, queue_size=50)
        self.raw_vel = TwistWithCovarianceStamped()

        self.pub_driverLeft = rospy.Publisher(self.topicControl_driverLeft, Driver_query, queue_size=50)
        self.driver1_query = Driver_query()

        self.pub_driverRight = rospy.Publisher(self.topicControl_driverRight, Driver_query, queue_size=50)
        self.driver2_query = Driver_query()
        self.driverRPM_query = RPM()

        self.odom_pub = rospy.Publisher('/odom_bldv', Odometry, queue_size=10)
        self.odom = Odometry()
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.lastTime_pub = time.time()
        self.time_pub = 1/self.frequency_control # s

        # -- 
        self.lastTime_speed1 = time.time()
        self.nowTime_speed1 = time.time()
        
        self.lastTime_speed2 = time.time()
        self.nowTime_speed2 = time.time()
        # -- find main driver.
        self.is_finded = 0
        self.mainDriver = 0
        # -- frequence pub raw vel.
        self.fre_rawVel = 25.
        self.cycle_rawVel = 1/self.fre_rawVel
        self.timeout = self.cycle_rawVel*4
        self.isNew_driver1 = 0
        self.isNew_driver2 = 0
        self.isNew_driver = 0
        # -- 
        self.is_exit = 1
        #-- 
        self.nowTime_cmdVel = time.time()
        self.timeout_cmdVel = 0.4
        self.is_timeout = 0
        # -- 
        self.max_deltaTime = 0.0
        # -- 
        self.max_rotation = 0.6 # rad/s

    def taskDriver_callback(self, data):
        self.task_driver = data

    def cmdVel_callback(self, data):
        self.cmd_vel = data
        self.nowTime_cmdVel = time.time()

    def driver1Respond_callback(self, data):
        self.driver1_respond = data
        self.nowTime_speed1 = time.time()
        self.isNew_driver1 = 1
        self.isNew_driver = 1

    def driver2Respond_callback(self, data):
        self.driver2_respond = data
        self.nowTime_speed2 = time.time()
        self.isNew_driver2 = 1
        self.isNew_driver = 1

    def constrain(self, val_in, val_compare1, val_compare2):
        val = 0.0
        val = max(val_in, val_compare1)
        val = min(val_in, val_compare2)
        return val

    def calculateRPM(self, linear_x, angular_z):
        linear_x_right = linear_x
        angular_x_right = angular_z

        linear_vel_x_mins = 0.0
        angular_vel_z_mins = 0.0
        tangential_vel = 0.0
        x_rpm = 0.0
        tan_rpm = 0.0
        # -- limit
        if (abs(linear_x_right) > self.linear_max):
            if (linear_x_right >= 0):
                linear_x_right = self.linear_max
            else:
                linear_x_right = -self.linear_max

        if (abs(angular_x_right) > self.angular_max):
            if (angular_x_right >= 0):
                angular_x_right = self.angular_max
            else:
                angular_x_right = -self.angular_max

        # convert m/s to m/min
        linear_vel_x_mins = linear_x_right * 60
        # convert rad/s to rad/min
        angular_vel_z_mins = angular_x_right * 60
        tangential_vel = angular_vel_z_mins * (self.distanceBetwentWheels / 2)

        x_rpm = linear_vel_x_mins / self.wheel_circumference
        tan_rpm = tangential_vel / self.wheel_circumference

        rpm_query = RPM()

        # calculate for the target motor RPM and direction
        # front-left motor
        rpm_query.motor1 = (x_rpm - tan_rpm)*self.transmission_ratio
        rpm_query.motor1 = self.constrain(rpm_query.motor1, -self.max_rpm, self.max_rpm)
        # front-right motor
        rpm_query.motor2 = (x_rpm + tan_rpm)*self.transmission_ratio
        rpm_query.motor2 = self.constrain(rpm_query.motor2, -self.max_rpm, self.max_rpm)
        return rpm_query

    def vvv(self, vel):
        t_incre = 0.5
        t_decre = 0.5

    def calculate_rawVelandOdom(self, rp1, rp2): # rp1,rp2: RPM | out: Velocities(m/s;rad/s)
        vel = TwistWithCovarianceStamped()
        odom = Odometry()

        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()

        # -- raw vel
        average_rps = ((rp2 + rp1)/2.)/60./self.transmission_ratio
        angular_rps = ((rp2 - rp1)/2.)/60./self.transmission_ratio

        linear_velocity = average_rps*self.wheel_circumference
        angular_velocity = (angular_rps*self.wheel_circumference)/(self.distanceBetwentWheels/2.)

        vel.twist.twist.linear.x = linear_velocity
        vel.twist.twist.angular.z = angular_velocity

        vel.twist.covariance[0] = 0.001
        vel.twist.covariance[7] = 0.001
        vel.twist.covariance[35] = 0.001

        vel.header.stamp = rospy.Time.now()
        vel.header.frame_id =  self.frame_id

        # -- Odometry
        delta_x = linear_velocity * dt * cos(self.theta)
        delta_y = linear_velocity * dt * sin(self.theta)
        delta_theta = angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        # print("x = %s, y = %s, theta = %s" %(self.x, self.y, degrees(self.theta)))
        odom_quat = quaternion_from_euler(0, 0, self.theta)

        odom.header.stamp = self.current_time
        # odom.header.frame_id = "odom_bldv"
        
        odom.pose.pose = Pose()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        
        # odom.child_frame_id = "base_link_bldv"
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        self.last_time = self.current_time

        return vel, odom

    def getVel_v1(self):
        if (self.isNew_driver == 1):
            vel_1 = self.driver1_respond.speed
            vel_2 = self.driver2_respond.speed
            self.raw_vel, self.odom = self.calculate_rawVelandOdom(vel_1, vel_2)
            self.pub_rawVel.publish(self.raw_vel)
            self.odom_pub.publish(self.odom)
            self.isNew_driver = 0

    def getVel(self):
        t1 = time.time() - self.nowTime_speed1
        t2 = time.time() - self.nowTime_speed2
        vel_1 = 0.
        vel_2 = 0.

        if (t1 < self.timeout and t2 < self.timeout): # -- check time out
            if (self.is_finded): # 
                # dong bo du lieu
                if (self.mainDriver == 1): # -- driver left
                    if (self.isNew_driver1 == 1): # neu co du lieu moi.
                        vel_1 = self.driver1_respond.speed
                        vel_2 = self.driver2_respond.speed
                        self.isNew_driver1 = 0

                else: # -- driver right
                    if (self.isNew_driver2 == 1): # neu co du lieu moi.
                        vel_1 = self.driver1_respond.speed
                        vel_2 = self.driver2_respond.speed
                        self.isNew_driver2 = 0

                self.raw_vel = self.calculate_rawVel(vel_1, vel_2)
                self.pub_rawVel.publish(self.raw_vel)
                return 1

            else:
                delta_dif = abs(self.nowTime_speed2 - self.nowTime_speed1)
                if (self.nowTime_speed2 >= self.nowTime_speed1):
                    if (delta_dif >= self.cycle_rawVel/2.):
                        self.is_finded = 1
                        self.mainDriver = 1
                    else:
                        self.is_finded = 1
                        self.mainDriver = 2
                else:
                    if (delta_dif >= self.cycle_rawVel/2.):
                        self.is_finded = 1
                        self.mainDriver = 2
                    else:
                        self.is_finded = 1
                        self.mainDriver = 1
                return 0
        else:
            self.is_finded = 0
            return -1

    def run_getVel(self):
        self.getVel_v1()
        # print ("ggg")

    def run(self):
        print ("Launch ALL!")
        while not rospy.is_shutdown(): # not self.shutdown_flag.is_set() or not rospy.is_shutdown()
            # -- Check Time out
            # t_timeout = (time.time() - self.nowTime_cmdVel)%60
            # if (t_timeout > self.timeout_cmdVel):
            # 	self.is_timeout = 1
            # else:
            # 	self.is_timeout = 0

            if (self.task_driver.data == 0): # -- Nothing
                self.driver1_query.task = 0
                self.driver2_query.task = 0

            elif (self.task_driver.data == 1): # -- Reset + Read status
                self.driver1_query.task = 1
                self.driver2_query.task = 1

            elif (self.task_driver.data == 2): # -- Read status
                self.driver1_query.task = 2
                self.driver2_query.task = 2

            # -- PUB
            t_pub = (time.time() - self.lastTime_pub)%60
            if (t_pub > self.time_pub):
                self.lastTime_pub = time.time()
                self.driverRPM_query = self.calculateRPM(self.cmd_vel.linear.x, self.cmd_vel.angular.z)

                self.driver1_query.modeStop = 1
                self.driver1_query.rotationSpeed = int(self.driverRPM_query.motor1)

                self.driver2_query.modeStop = 1
                self.driver2_query.rotationSpeed = int(self.driverRPM_query.motor2)
                
                if (self.is_timeout):
                    self.pub_driverLeft.publish(Driver_query())
                    self.pub_driverRight.publish(Driver_query())
                else: # -- ok
                    self.pub_driverLeft.publish(self.driver1_query)
                    self.pub_driverRight.publish(self.driver2_query)
            self.rate.sleep()
        self.is_exit = 0
        print('program stopped')

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
        programer = kinematic(1)
        programer.start()

        # Keep the main thread running, otherwise signals are ignored.
        while programer.is_exit:
            programer.run_getVel()
            time.sleep(0.001)

    except ServiceExit:
        programer.shutdown_flag.set()
        programer.join()
    print('Exiting main program')

if __name__ == '__main__':
    main()
