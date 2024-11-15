#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose

def getPose(robot_odom_msg):

    # doi quaternion -> rad    
    quaternion1 = (robot_odom_msg.orientation.x, robot_odom_msg.orientation.y,\
                    robot_odom_msg.orientation.z, robot_odom_msg.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion1)
    theta = euler[2]
    # print theta #rad


    quaternion2 = tf.transformations.quaternion_from_euler(0, 0, theta)
    print quaternion2



if __name__ == '__main__':
    rospy.init_node('convert')
    sub_odom_robot = rospy.Subscriber('/robot_pose', Pose, getPose, queue_size = 1)
    
    rospy.spin()
