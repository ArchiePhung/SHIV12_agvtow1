#!/usr/bin/env python3

import socketio
import roslib

import json 

# get ip
import os                                                                                                                                                           
import re 

import sys
import struct
import time
from decimal import *
import math
import rospy
from datetime import datetime

from sti_msgs.msg import NN_cmdRequest     # request to client udp
from sti_msgs.msg import NN_infoRequest     # request to client udp
from sti_msgs.msg import NN_infoRespond     # respond from client udp

from sti_msgs.msg import FL_infoRespond
from sti_msgs.msg import FL_cmdRespond
from sti_msgs.msg import FL_infoRequest
from sti_msgs.msg import FL_cmdRequest

# import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped, Quaternion

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


sio = socketio.Client()

@sio.on('Server-request-AGV')
def on_message(data):
    print('I received a message!')
    print(data)

@sio.event
def connect():
    print("I'm connected!")

@sio.event
def connect_error(data):
    print("The connection failed!")

@sio.event 
def disconnect():
    print("I'm disconnected!")

def callback(data):
    global sio
    quaternion1 = (data.pose.orientation.x, data.pose.orientation.y,\
                    data.pose.orientation.z, data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion1)
    x = round(data.pose.position.x, 3)
    y = round(data.pose.position.y, 3)
    z = round(euler[2], 3)
    data = {"id": 1, "name": "lagv", "enable": 0, "status": "diem1", "x": x, "y": y, "r": z}
    json_object = json.dumps(data, indent = 4)
    sio.emit("AGV-respond-Server", json_object)
    
def listener():
    global sio
    sio.connect('http://192.168.1.99:3000')
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('testSocket', anonymous=True)

    rospy.Subscriber("/robotPose_nav", PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()