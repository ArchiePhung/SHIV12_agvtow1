#!/usr/bin/python3

import os 
import time
import rospy

import sys
import struct
import string
import roslib

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose, Twist, TwistWithCovarianceStamped, Point

from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs, acos, radians, degrees
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class calibOrigin():
    def __init__(self):
        rospy.init_node('calib_origin', anonymous=True)
        self.rate = 30
        self.rate = rospy.Rate(self.rate)

        rospy.Subscriber('/robotPose_lidarLOC', PoseStamped, self.callback_getPose, queue_size = 20)
        self.is_pose_robot = False
        self.poseRbMa = Pose()
        self.theta_rb = 0.0

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
        self.time_tr = rospy.get_time()
        self.rate_cmdvel = 25

        self.process = 1
        self.count = 0
        self.saveTimeCount = rospy.get_time()

        self.angle_giam_toc = radians(30.)

        self.point0 = Point()
        self.point180 = Point()
        self.pointP90 = Point()
        self.pointN90 = Point()


        self.clicked = True
        self.finish_program = False

    def callback_getPose(self, data):
        self.is_pose_robot = True
        self.poseRbMa = data.pose
        quata = ( self.poseRbMa.orientation.x,\
                self.poseRbMa.orientation.y,\
                self.poseRbMa.orientation.z,\
                self.poseRbMa.orientation.w )
        euler = euler_from_quaternion(quata)
        self.theta_rb = euler[2]

    def stop(self):
        for i in range(2):
            self.pub_cmd_vel.publish(Twist())


    def pub_cmdVel(self, twist , rate):
        if rospy.get_time() - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = rospy.get_time()
            self.pub_cmd_vel.publish(twist)
        else :
            pass


    def turn_ar(self, theta, tol_theta, vel_rot):
        if fabs(theta) > tol_theta: # +- 10 do
            if theta > 0: #quay trai
                # print "b"
                if fabs(theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhh')
                    vel_th = (fabs(theta)/self.angle_giam_toc)*vel_rot
                else:
                    vel_th = vel_rot

                if vel_th < 0.1:
                    vel_th = 0.1

                # vel_th = fabs(theta) + 0.1
                # if vel_th > vel_rot : vel_th = vel_rot
                return vel_th

            elif theta < 0: #quay phai , vel_z < 0
                # print "a"
                if fabs(theta) <= self.angle_giam_toc:
                    # print('hhhhhhhhhhhh')
                    vel_th = (fabs(theta)/self.angle_giam_toc)*(-vel_rot)
                else:
                    vel_th = -vel_rot

                if vel_th > -0.1:
                    vel_th = -0.1

                # vel_th = -fabs(theta) - 0.1
                # if vel_th < -vel_rot : vel_th = -vel_rot
                return vel_th
                # buoc = 1

        else : 
            return 10


    def run(self):
        while not self.finish_program:
            if self.is_pose_robot:
                if self.process == 1: # quay robot ve huong 0 do
                    angle_target = 0.
                    d_angle = angle_target - self.theta_rb
                    gt = self.turn_ar(d_angle, radians(1.), 0.2)
                    if gt == 10:
                        self.stop()
                        self.process = 2
                        self.saveTimeCount = rospy.get_time()

                    else:
                        twist = Twist()
                        twist.angular.z = gt
                        self.pub_cmdVel(twist, self.rate_cmdvel)

                elif self.process == 2: # lay tb toa do 10 lan
                    dt = rospy.get_time() - self.saveTimeCount
                    if dt <= 3:
                        self.point0.x = self.point0.x + self.poseRbMa.position.x
                        self.point0.y = self.point0.y + self.poseRbMa.position.y

                        self.count = self.count + 1
                    else:
                        self.point0.x = self.point0.x/self.count
                        self.point0.y = self.point0.y/self.count
                        self.count = 0
                        self.process = 3
                        if self.clicked:
                            print("OKE, NEXT")
                            input()

                elif self.process == 3: # quay robot ve huong 90 do
                    angle_target = radians(90.)
                    d_angle = angle_target - self.theta_rb
                    gt = self.turn_ar(d_angle, radians(1.), 0.2)
                    if gt == 10:
                        self.stop()
                        self.process = 4
                        self.saveTimeCount = rospy.get_time()

                    else:
                        twist = Twist()
                        twist.angular.z = gt
                        self.pub_cmdVel(twist, self.rate_cmdvel)

                elif self.process == 4:
                    dt = rospy.get_time() - self.saveTimeCount
                    if dt <= 3:
                        self.pointP90.x = self.pointP90.x + self.poseRbMa.position.x
                        self.pointP90.y = self.pointP90.y + self.poseRbMa.position.y

                        self.count = self.count + 1
                    else:
                        self.pointP90.x = self.pointP90.x/self.count
                        self.pointP90.y = self.pointP90.y/self.count
                        self.count = 0
                        self.process = 5
                        if self.clicked:
                            print("OKE, NEXT")
                            input()

                elif self.process == 5: # quay robot ve huong 180 do
                    angle_target = radians(180.)
                    d_angle = angle_target - self.theta_rb
                    gt = self.turn_ar(d_angle, radians(1.), 0.2)
                    if gt == 10:
                        self.stop()
                        self.process = 6
                        self.saveTimeCount = rospy.get_time()

                    else:
                        twist = Twist()
                        twist.angular.z = gt
                        self.pub_cmdVel(twist, self.rate_cmdvel)

                elif self.process == 6:
                    dt = rospy.get_time() - self.saveTimeCount
                    if dt <= 3:
                        self.point180.x = self.point180.x + self.poseRbMa.position.x
                        self.point180.y = self.point180.y + self.poseRbMa.position.y

                        self.count = self.count + 1
                    else:
                        self.point180.x = self.point180.x/self.count
                        self.point180.y = self.point180.y/self.count
                        self.count = 0
                        self.process = 7
                        if self.clicked:
                            print("OKE, NEXT")
                            input()

                elif self.process == 7: # quay robot ve huong -90 do
                    angle_target = radians(-90.)
                    d_angle = angle_target - self.theta_rb
                    gt = self.turn_ar(d_angle, radians(1.), 0.2)
                    if gt == 10:
                        self.stop()
                        self.process = 8
                        self.saveTimeCount = rospy.get_time()

                    else:
                        twist = Twist()
                        twist.angular.z = gt
                        self.pub_cmdVel(twist, self.rate_cmdvel)

                elif self.process == 8:
                    dt = rospy.get_time() - self.saveTimeCount
                    if dt <= 3:
                        self.pointN90.x = self.pointN90.x + self.poseRbMa.position.x
                        self.pointN90.y = self.pointN90.y + self.poseRbMa.position.y

                        self.count = self.count + 1
                    else:
                        self.pointN90.x = self.pointN90.x/self.count
                        self.pointN90.y = self.pointN90.y/self.count
                        self.count = 0
                        self.process = 9

                elif self.process == 9:
                    print('pose 0: \n', self.point0)
                    print('pose 90: \n', self.pointP90)
                    print('pose 180: \n', self.point180)
                    print('pose -90: \n', self.pointN90)

                    print('dental X1 = ', (self.point0.x - self.point180.x)/2.)
                    print('dental Y1 = ', (self.point0.y - self.point180.y)/2.)

                    print('dental X2 = ', (self.pointP90.x - self.pointN90.x)/2.)
                    print('dental Y2 = ', (self.pointP90.y - self.pointN90.y)/2.)

                    self.finish_program = True

            self.rate.sleep()
            
def main():

    program = calibOrigin()
    program.run()
    print('End Program!')
        #pass

if __name__ == '__main__':
    main()
