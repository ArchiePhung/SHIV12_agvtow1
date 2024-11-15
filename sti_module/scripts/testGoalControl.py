#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from math import sin , cos , pi , atan2, radians, sqrt, pow, degrees, fabs
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped, PoseStamped
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler

shape_move = 1
timeReadPose = time.time()
pose_robot = PoseStamped()
is_robotPose = 0

msg_vel = Twist()

def callback_robotPose(data):
    global pose_robot, is_robotPose, shape_move, timeReadPose
    if is_robotPose == 0:
        pose_robot = data
    elif is_robotPose:
        if time.time() - timeReadPose > 0.5:
            trans = sqrt(pow(data.pose.position.x - pose_robot.pose.position.x,2) + pow(data.pose.position.y - pose_robot.pose.position.y,2))
            angle_now = euler_from_quaternion((data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))[2]
            angle_pre = euler_from_quaternion((pose_robot.pose.orientation.x, pose_robot.pose.orientation.y, pose_robot.pose.orientation.z, pose_robot.pose.orientation.w))[2]
            theta = angle_now - angle_pre
            if fabs(theta) >= pi:
                theta_t = (2*pi - fabs(theta))
                if theta > 0:
                    theta = -theta_t
                else:
                    theta = theta_t

            print(trans, theta)
            if fabs(theta) > radians(1.5) and trans > 0.04:
                if theta > 0:
                    shape_move = 2
                else:
                    shape_move = 3
            else:
                shape_move = 1

            pose_robot = data
            timeReadPose = time.time()

    print(shape_move)
    is_robotPose = 1

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/robotPose_lidarLOC", PoseStamped, callback_robotPose)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()