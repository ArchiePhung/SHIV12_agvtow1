#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import time
import signal

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped, Quaternion

from sti_msgs.msg import EncoderData
from std_msgs.msg import Int8

from math import sin , cos , pi , atan2, degrees, radians

from tf.transformations import quaternion_from_euler


class OdomEncoder():
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('odom_encoder', anonymous= False) # False
        self.rate = rospy.Rate(30)

        # -- parameter
        self.wheel_circumference = rospy.get_param("wheel_circumference", 0.47036)
        self.distanceBetwentWheels = rospy.get_param("distanceBetwentWheels", 0.562)
        self.numTickPerRound = rospy.get_param("numTickPerRound", 1600)

        # -----------------
        rospy.Subscriber("/encoder_respond", EncoderData, self.encoder_callback)

        # -----------------
        self.pub_odomEncoder = rospy.Publisher("/odom_encoder", Odometry, queue_size = 50)
        self.odomEncoder = Odometry()

        # -----------------
        self.pub_resetEncoder = rospy.Publisher("/encoder_request", Int8, queue_size = 50)

        # -- 
        self.tick_encoderLeft = None
        self.tick_encoderRight = None

        self.delta_theta = 0.
        self.delta_s = 0.

        # -- save param
        self.pre_x = 0.
        self.pre_y = 0.
        self.pre_theta = 0.

        self.pre_measurement_time = rospy.Time.now()

        self.start = 0
        self.reset_encoder()

        rospy.spin()

    def reset_encoder(self):
        for i in range(5):
            self.pub_resetEncoder.publish(Int8(1))
            rospy.sleep(0.1)

        for i in range(5):
            self.pub_resetEncoder.publish(Int8(0))
            rospy.sleep(0.1)

    def encoder_callback(self, data):     
        time_measurenment = rospy.Time.now()
           
        if self.start == 0:
            if data.encoder_left_val == 0 and data.encoder_right_val == 0:
                self.tick_encoderLeft = data.encoder_left_val
                self.tick_encoderRight = data.encoder_right_val
                self.start = 1
            return
        
        # -- 
        delta_enLeft = data.encoder_left_val - self.tick_encoderLeft
        delta_enRight = data.encoder_right_val - self.tick_encoderRight

        # print(delta_enLeft, delta_enRight)

        self.tick_encoderLeft = data.encoder_left_val
        self.tick_encoderRight = data.encoder_right_val

        wl = (delta_enLeft/self.numTickPerRound)* self.wheel_circumference
        wr = (delta_enRight/self.numTickPerRound)* self.wheel_circumference

        self.delta_theta = (wr - wl) / self.distanceBetwentWheels
        self.delta_s = (wr + wl) / 2.

        theta = self.pre_theta + self.delta_theta
        x = self.pre_x + (cos(theta)*self.delta_s)
        y = self.pre_y + (sin(theta)*self.delta_s)

        self.pre_theta = theta 
        self.pre_x=x
        self.pre_y=y

        # print(x, y, degrees(theta))
        dt = (time_measurenment - self.pre_measurement_time).to_sec()

        odom = Odometry()
        odom.header.stamp = time_measurenment
        # odom.header.frame_id = 'odom_encoder'
        # odom.child_frame_id = "base_link_encoder"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        odom_quat = quaternion_from_euler(0., 0., theta)
        msg_odom_quat = Quaternion(*odom_quat)
        odom.pose.pose.orientation = msg_odom_quat
        
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.delta_theta/dt
        
        if(data.encoder_left_val > 0 and data.encoder_right_val < 0) or (data.encoder_left_val < 0 and data.encoder_right_val > 0):
            odom.twist.twist.linear.x = 0
        else:
            odom.twist.twist.linear.x = self.delta_s/dt

        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.001

        odom.twist.covariance[0] = 0.0001
        odom.twist.covariance[7] = 0.0001
        odom.twist.covariance[35] = 0.0001

        self.pub_odomEncoder.publish(odom)
        self.pre_measurement_time = time_measurenment


def main():
	print('Starting main program')

	program = OdomEncoder()
	print('Exiting main program')	

if __name__ == '__main__':
    main()	






