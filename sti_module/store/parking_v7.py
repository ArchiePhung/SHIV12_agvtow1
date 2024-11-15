#!/usr/bin/env python

import rospy
import numpy as np
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Twist ,Pose

import math
import time
import threading
import signal

name_tag = 0
tt_x = .0
tt_z = .0
tt_g = .0
is_marker_pose_received = False  

class ReadApriltag(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        
        self.shutdown_flag = threading.Event()
        rospy.init_node('tag_detections_filter')

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.sub_info_marker = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.cbGetMarkerOdom, queue_size = 1)
        self.pub_filter = rospy.Publisher('/tag_detections_filter', Pose, queue_size=1)

        self.id_tag = 0
        self.k1 = self.x_ht1 = self.x_truoc1 = 0.
        self.k2 = self.x_ht2 = self.x_truoc2 = 0.
        self.k3 = self.x_ht3 = self.x_truoc3 = 0.

    def my_kalman1(self,r,p,q,input):
        self.k1=p/(p+r)
        self.x_ht1 = self.x_truoc1 + self.k1*(input-self.x_truoc1)
        p=(1-self.k1)*p+ math.fabs(self.x_truoc1-self.x_ht1)*q
        self.x_truoc1 = self.x_ht1    
        return self.x_ht1
    
    def my_kalman2(self,r,p,q,input):
        self.k2=p/(p+r)
        self.x_ht2 = self.x_truoc2 + self.k2*(input-self.x_truoc2)
        p=(1-self.k2)*p+ math.fabs(self.x_truoc2-self.x_ht2)*q
        self.x_truoc2 = self.x_ht2      
        return self.x_ht2

    def my_kalman3(self,r,p,q,input):
        self.k3=p/(p+r)
        self.x_ht3 = self.x_truoc3 + self.k3*(input-self.x_truoc3)
        p=(1-self.k3)*p+ math.fabs(self.x_truoc3-self.x_ht3)*q
        self.x_truoc3 = self.x_ht3    
        return self.x_ht3

    def cbGetMarkerOdom(self, markers_odom_msg): # 5Hz
        # print "cbGetMarkerOdom"
        global is_marker_pose_received 
        sl_tag = len(markers_odom_msg.detections)
        if sl_tag != 0 :
            if is_marker_pose_received == False:
                is_marker_pose_received = True

            marker_odom = markers_odom_msg.detections[0]
            self.id_tag = markers_odom_msg.detections[0].id[0]

            self.tag_g = math.atan2(marker_odom.pose.pose.pose.position.x,\
                                    marker_odom.pose.pose.pose.position.z)

    def fnFilterDataTag(self,id_tag): 
        global is_marker_pose_received
        
        name_tag = "tag_" + str(int(id_tag))
        if is_marker_pose_received == True :
            self.tf_listener.waitForTransform(name_tag,"base_footprint",rospy.Time(),rospy.Duration(1))
            position, quaternion = self.tf_listener.lookupTransform(name_tag, "base_footprint", rospy.Time())
            quaternion = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]

            x = self.my_kalman1(10.0, 1.0, 0.01, position[0])
            z = self.my_kalman2(10.0, 1.0, 0.01, position[2])
            g = self.my_kalman3(10.0, 1.0, 0.01, theta) # radian
            is_marker_pose_received = False

        return x, z, g

    def run(self):
        global name_tag , tt_x, tt_z, tt_g 
        while not self.shutdown_flag.is_set():
            data_filter = Pose()
            # data_filter.position.x = 
            tt_x , tt_z, tt_g = self.fnFilterDataTag(self.id_tag)
            print "tt_x= %f , tt_z= %f, tt_g= %f" %(tt_x, tt_z, tt_g)
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

        j2 = ReadApriltag(1)
        j2.start()

 
        # Keep the main thread running, otherwise signals are ignored.
        while True:
            time.sleep(0.5)
 
    except ServiceExit:
        # Terminate the running threads.
        # Set the shutdown flag on each thread to trigger a clean shutdown of each thread.
        
        j2.shutdown_flag.set()
        # Wait for the threads to close...
        
        j2.join()

 
    print('Exiting main program')

if __name__ == '__main__':
    main()