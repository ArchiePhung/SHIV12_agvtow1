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


class StiAutomaticParking(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.shutdown_flag = threading.Event()
        rospy.init_node('sti_automatic_parking')

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.sub_odom_robot = rospy.Subscriber('/odom', Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.sub_info_marker = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.cbGetMarkerOdom, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_log = rospy.Publisher('/data_log', Pose, queue_size=1)

        self.pub = rospy.Publisher('/parking_client', Pose, queue_size=10) 
        rospy.Subscriber("/parking_server", Pose, self.callSup)

        self.is_odom_received = False
        self.is_marker_pose_received = False   
        rospy.on_shutdown(self.fnShutDown)

        self.tt_x = .0
        self.tt_z = .0
        self.tt_g = .0
        self.tag_g = .0
        self.odom_x = .0
        self.odom_y = .0
        self.odom_g = .0

        self.enable_parking = False
        self.idtag_parking = 0
        self.MARKER_ID_DETECTION = 11
        self.solanhoatdong = 0
        self.dolech_x_CamVsRObot = -0.025 # m : lech Trai
        self.dolech_y_CamVsRObot = 0.7 # m
        self.khoangCachDocTag = 1.6
    
    def write_log(self,solan,isTag,x,y,g):
        log = Pose()
        log.position.x = solan
        log.position.y = isTag # 0 : k nhin thay Tag , 1 : co Tag

        log.orientation.x = x
        log.orientation.y = y
        log.orientation.z = g
        self.pub_log.publish(log)

    def park_info(self, info):
        park = Pose()
        park.position.x = info # 0: nothing , 1-9 : running, 10 : done
        self.pub.publish(park)
    
    def callSup(self,data):
        self.idtag_parking = data.position.x
        if self.idtag_parking != 0 :
            self.enable_parking = True   
        else : self.enable_parking = False  

    def cbGetRobotOdom(self, robot_odom_msg):
        if self.is_odom_received == False:
            self.is_odom_received = True 
        quaternion = (robot_odom_msg.pose.pose.orientation.x, robot_odom_msg.pose.pose.orientation.y,\
                      robot_odom_msg.pose.pose.orientation.z, robot_odom_msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        self.odom_x = robot_odom_msg.pose.pose.position.x
        self.odom_y = robot_odom_msg.pose.pose.position.y
        # self.odom_g = (theta*180)/np.pi
        self.odom_g = theta
        # http://docs.ros.org/jade/api/tf/html/python/transformations.html#references
    
    def cbGetMarkerOdom(self, markers_odom_msg): # 5Hz
        # print "cbGetMarkerOdom"
        sl_tag = len(markers_odom_msg.detections)
        for sl_tag in range(sl_tag):            
            if markers_odom_msg.detections[sl_tag-1].id[0] == self.MARKER_ID_DETECTION:
                if self.is_marker_pose_received == False:
                    self.is_marker_pose_received = True
       
                marker_odom = markers_odom_msg.detections[sl_tag-1]

                self.tag_g = math.atan2(marker_odom.pose.pose.pose.position.x,\
                                        marker_odom.pose.pose.pose.position.z)

    def fnGetDataTag(self,id_tag,sl_loc): 
        tt_x = tt_z = tt_g = .0
        dem = 0
        name_tag = "tag_" + str(int(id_tag))
        for dem in range(sl_loc):
            while self.is_marker_pose_received != True :
                dem = dem
            else:
                self.tf_listener.waitForTransform(name_tag,"base_footprint",rospy.Time(),rospy.Duration(1))
                position, quaternion = self.tf_listener.lookupTransform(name_tag, "base_footprint", rospy.Time())
                # theta2 = math.atan2(position[0],position[2]) 
                # print position[0] ,position[2], theta2
                quaternion = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
                theta = tf.transformations.euler_from_quaternion(quaternion)[1]
                tt_x = tt_x + position[0]
                tt_z = tt_z + position[2]
                tt_g = tt_g + theta # radian
                self.is_marker_pose_received = False
                dem = dem + 1

        tt_x = tt_x / (sl_loc)
        tt_z = tt_z / (sl_loc)
        tt_g = tt_g / (sl_loc)
        return tt_x, tt_z, tt_g

    def fnGetDataTag_notFilter(self,id_tag): 
        tt_x = tt_z = tt_g = .0
        name_tag = "tag_" +  str(int(id_tag))
        self.tf_listener.waitForTransform(name_tag,"base_footprint",rospy.Time(),rospy.Duration(1))
        position, quaternion = self.tf_listener.lookupTransform(name_tag, "base_footprint", rospy.Time())
        quaternion = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
        theta = tf.transformations.euler_from_quaternion(quaternion)[1]
        tt_x = tt_x + position[0]
        tt_z = tt_z + position[2]
        tt_g = tt_g + theta # radian
        self.is_marker_pose_received = False

        return tt_x, tt_z, tt_g

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist()) 

    def fnCalcDistPoints(self, x1, x2, y1, y2):
        print "fnCalcDistPoints"
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def run(self):
        loop_rate = rospy.Rate(10) # 10hz
        buoc = -1
        while not self.shutdown_flag.is_set():
            self.park_info(buoc) 
          #Test
            if buoc == 0 :
                # print self.fnGetDataTag(self.MARKER_ID_DETECTION,10)
                # print "odom_g = ", self.odom_g
                self.tt_x, self.tt_z, self.tt_g = self.fnGetDataTag_notFilter(self.MARKER_ID_DETECTION) # rad
                # print "tt_x= %f , tt_z= %f, tt_g= %f" %(self.tt_x, self.tt_z, self.tt_g)
                print "tag_g: ", self.tag_g
                # rospy.spin()

          # Nhan thong tin tu server
            if buoc == -1 :
                if self.enable_parking == True:
                    self.MARKER_ID_DETECTION = self.idtag_parking
                    self.enable_parking = False
                    buoc = 1 

          #B1: Huong camera ve Apriltag
            print "buoc: ", buoc
            if buoc == 1 :
                # rospy.sleep(1)
                print "tag_g: ", self.tag_g
                twist = Twist()
                if math.fabs(self.tag_g) > 0.1 : #2do
                    if self.tag_g > 0 : twist.angular.z = -0.15
                    else :              twist.angular.z = 0.15
                    self.pub_cmd_vel.publish(twist)
                else:
                    self.pub_cmd_vel.publish(Twist())
                    buoc = 2
                    # rospy.spin()

          #B2: Tinh goc can quay, tt_x , tt_z
            print "buoc: ", buoc
            if buoc == 2 : 

                self.tt_x, self.tt_z, self.tt_g = self.fnGetDataTag(self.MARKER_ID_DETECTION,20) # rad
                print "tt_x= %f , tt_z= %f, tt_g= %f" %(self.tt_x, self.tt_z, self.tt_g)
                print "odom_g = ", self.odom_g
              
              # tinh z
                if math.fabs(self.tt_z - self.khoangCachDocTag) > 0.1 : # dung sai +- 10cm
                    z_target = math.fabs(self.tt_z - self.khoangCachDocTag)
                    buoc = 22

              # tinh x can di
                x_target = self.tt_x - self.dolech_x_CamVsRObot
                if math.fabs(x_target) < 0.02:
                    odom_x_ht = self.odom_x
                    odom_y_ht = self.odom_y
                    buoc = 6
                else :
                  # tinh goc quay
                    quayTrai = quayPhai = False
                    goc_canquay = math.fabs(self.tt_g)
                    if x_target > 0 : # quay trai ( + tang , - giam)
                        print "quay trai"
                        quayTrai = True
                        if self.odom_g > 0 : 
                            goc_taget= self.odom_g + goc_canquay
                            if goc_taget > np.pi : 
                                goc_bu = goc_taget - np.pi 
                                goc_taget = goc_bu - np.pi 
                        else : 
                            goc_taget= self.odom_g + goc_canquay
                            # if goc_taget < -np.pi :
                            #     goc_taget = goc_taget + 2*np.pi 

                    else : # quay phai ( + giam , - tang)
                        print "quay phai"
                        quayPhai = True
                        if self.odom_g > 0 : 
                            goc_taget= self.odom_g - goc_canquay
                            # if goc_taget < -np.pi : 
                            #     goc_bu = goc_taget + np.pi 
                            #     goc_taget = goc_bu + np.pi 
                        else : 
                            goc_taget= self.odom_g - goc_canquay
                            if goc_taget < -np.pi :
                                goc_taget = goc_taget + 2*np.pi

                    print "goc_taget= " , goc_taget
                    print "x_target: ", x_target

                    buoc = 3
                    # rospy.spin()

          #B22: Tinh tien bu z = khoangCachDocTag ( 1.6m )
            print "buoc: ", buoc
            if buoc == 22 : 
                self.tt_x, self.tt_z, self.tt_g = self.fnGetDataTag_notFilter(self.MARKER_ID_DETECTION) # rad
                print "tt_x= %f , tt_z= %f, tt_g= %f" %(self.tt_x, self.tt_z, self.tt_g)
                twist = Twist()
                s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                print "Tien bu = z_target: ", z_target
                print "s: ", s
                print "tag_g: ", self.tag_g
                if math.fabs(s) < (math.fabs(z_target)-self.dolech_y_CamVsRObot) : 
                    twist.linear.x = 0.2
                    if self.tag_g > 0.02  : twist.angular.z = -0.05
                    if self.tag_g < -0.02 : twist.angular.z = 0.05
                    self.pub_cmd_vel.publish(twist)
                    
                else : 
                    self.pub_cmd_vel.publish(Twist())
                    buoc = 3
                    # rospy.spin() 

          #B3: Quay vuong goc 
            print "buoc: ", buoc
            if buoc == 3 :
                print "odom_g = ", self.odom_g
                print "goc_taget= " , goc_taget
                print "x_target: ", x_target
                twist = Twist()
                if(goc_taget > 0):
                    print "a"
                    if math.fabs(self.odom_g - goc_taget) > 0.04 :
                        print "b"
                        if x_target < 0 : twist.angular.z = -0.3
                        else :             twist.angular.z = 0.3
                        self.pub_cmd_vel.publish(twist)
                    else:
                        print "c"
                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        buoc = 4 
                        # rospy.spin()

                else:
                    print "m"
                    if math.fabs(self.odom_g - goc_taget) > 0.04 :
                        if x_target < 0 : twist.angular.z = -0.3
                        else :            twist.angular.z = 0.3
                        self.pub_cmd_vel.publish(twist)
                    else:
                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        buoc = 4 
                        # rospy.spin() 

          #B4: Di chuyen chinh giua Apriltag   
            print "buoc: ", buoc
            if buoc == 4 :
                twist = Twist()
                s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                print "x_target: ", x_target
                print "s: ", s
                if math.fabs(s) < (math.fabs(x_target)) : 
                    twist.linear.x = 0.05
                    self.pub_cmd_vel.publish(twist)
                    
                else : 
                    self.pub_cmd_vel.publish(Twist())
                    buoc = 55
                    # rospy.spin() 

          #B55: tinh goc quay lai
            print "buoc: ", buoc
            if buoc == 55 :
                print "odom_g   = ", self.odom_g
                print "x_target: ", x_target
                if quayTrai == True:
                    print "quay phai"
                    if self.odom_g > 0 : 
                        goc_taget = self.odom_g - np.pi/2
                        # if goc_taget < -np.pi : 
                        #     goc_bu = goc_taget + np.pi 
                        #     goc_taget = goc_bu + np.pi 
                    else : 
                        goc_taget = self.odom_g - np.pi/2
                        if goc_taget < -np.pi :
                            goc_taget = goc_taget + 2*np.pi


                if quayPhai == True:    
                    print "quay trai"   
                    if self.odom_g > 0 : 
                        goc_taget= self.odom_g + np.pi/2
                        if goc_taget > np.pi : 
                            goc_bu = goc_taget - np.pi 
                            goc_taget = goc_bu - np.pi 
                    else : 
                        goc_taget= self.odom_g + np.pi/2

                print "goc_taget_quaylai= " , goc_taget  
                buoc = 555
                # rospy.spin()

          #B5: quay nguoc lai
            print "buoc: ", buoc
            if buoc == 555 :
                print "odom_g   = ", self.odom_g
                print "goc_taget= " , goc_taget
                print "x_target: ", x_target
                print "s: ", s
                twist = Twist()
                if(goc_taget > 0):
                    print "a"
                    if math.fabs(self.odom_g - goc_taget) > 0.03 :
                        print "b"
                        if quayPhai == True : twist.angular.z = 0.3
                        if quayTrai == True : twist.angular.z = -0.3
                        self.pub_cmd_vel.publish(twist)
                    else:
                        print "c"
                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        buoc = 6 
                        # rospy.spin()

                else:
                    print "m"
                    if math.fabs(self.odom_g - goc_taget) > 0.03 :
                        if quayPhai == True : twist.angular.z = 0.3
                        if quayTrai == True : twist.angular.z = -0.3
                        self.pub_cmd_vel.publish(twist)
                    else:
                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        buoc = 6
                        # rospy.spin()
            
          #B6: tinh chinh goc
            print "buoc: ", buoc
            if buoc == 6 : 
                # rospy.sleep(1)
                print "tag_g: ", self.tag_g
                print "x_target: ", x_target
                # print "s: ", s
                twist = Twist()
                if math.fabs(self.tag_g) > 0.020 : #2do
                    if self.tag_g > 0 : twist.angular.z = -0.1
                    else :              twist.angular.z = 0.1
                    self.pub_cmd_vel.publish(twist)

                elif math.fabs(self.tag_g) < 0.020 and math.fabs(self.tag_g) > 0.010 : #2do
                    if self.tag_g > 0 : twist.angular.z = -0.06
                    else :              twist.angular.z = 0.06
                    self.pub_cmd_vel.publish(twist)

                else:
                    self.pub_cmd_vel.publish(Twist())
                    buoc = 7
                    # rospy.spin()

          #B7: tinh kc di vao
            print "buoc: ", buoc
            if buoc == 7 : 
                self.tt_x, self.tt_z, self.tt_g = self.fnGetDataTag(self.MARKER_ID_DETECTION,20) # rad
                print "tt_x= %f , tt_z= %f, tt_g= %f" %(self.tt_x, self.tt_z, self.tt_g)

                z_target = self.tt_z
                print "z_target: ", z_target
                # tinh x can di
                x_target = self.tt_x - self.dolech_x_CamVsRObot
                if math.fabs(x_target) > 0.02: # dung sai +- 2cm
                    buoc = 77
                else : 
                    buoc = 8
                # rospy.spin()
          
                    #B2: Tinh goc can quay, tt_x , tt_z
           
           #B77 : Tinh laij buoc 2
            print "buoc: ", buoc
            if buoc == 77 : 

                # self.tt_x, self.tt_z, self.tt_g = self.fnGetDataTag(self.MARKER_ID_DETECTION,20) # rad
                print "tt_x= %f , tt_z= %f, tt_g= %f" %(self.tt_x, self.tt_z, self.tt_g)
                print "odom_g = ", self.odom_g
              
              # tinh z
                if math.fabs(self.tt_z - self.khoangCachDocTag) > 0.1 : # dung sai +- 10cm
                    z_target = math.fabs(self.tt_z - self.khoangCachDocTag)
                    buoc = 22

              # tinh x can di
                x_target = self.tt_x - self.dolech_x_CamVsRObot
                if math.fabs(x_target) < 0.02:
                    buoc = 6
                else :
                  # tinh goc quay
                    quayTrai = quayPhai = False
                    goc_canquay = math.fabs(self.tt_g)
                    if x_target > 0 : # quay trai ( + tang , - giam)
                        print "quay trai"
                        quayTrai = True
                        if self.odom_g > 0 : 
                            goc_taget= self.odom_g + goc_canquay
                            if goc_taget > np.pi : 
                                goc_bu = goc_taget - np.pi 
                                goc_taget = goc_bu - np.pi 
                        else : 
                            goc_taget= self.odom_g + goc_canquay
                            # if goc_taget < -np.pi :
                            #     goc_taget = goc_taget + 2*np.pi 

                    else : # quay phai ( + giam , - tang)
                        print "quay phai"
                        quayPhai = True
                        if self.odom_g > 0 : 
                            goc_taget= self.odom_g - goc_canquay
                            # if goc_taget < -np.pi : 
                            #     goc_bu = goc_taget + np.pi 
                            #     goc_taget = goc_bu + np.pi 
                        else : 
                            goc_taget= self.odom_g - goc_canquay
                            if goc_taget < -np.pi :
                                goc_taget = goc_taget + 2*np.pi

                    print "goc_taget= " , goc_taget
                    print "x_target: ", x_target

                    buoc = 3
                    # rospy.spin()
                
          #B8: tinh tien di vao
            print "buoc: ", buoc
            if buoc == 8 :
                self.tt_x, self.tt_z, self.tt_g = self.fnGetDataTag_notFilter(self.MARKER_ID_DETECTION) # rad
                print "tt_x= %f , tt_z= %f, tt_g= %f" %(self.tt_x, self.tt_z, self.tt_g)
                twist = Twist()
                s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                print "z_target: ", z_target
                print "s: ", s
                print "tag_g: ", self.tag_g
                if math.fabs(s) < (math.fabs(z_target)-self.dolech_y_CamVsRObot) : 
                    twist.linear.x = 0.2
                    if self.tag_g > 0.015  : twist.angular.z = -0.05
                    if self.tag_g < -0.015 : twist.angular.z = 0.05
                    self.pub_cmd_vel.publish(twist)
                    
                else : 
                    self.pub_cmd_vel.publish(Twist())
                    buoc = 10
                    # rospy.spin() 

          #B10: publish Log
            print "buoc: ", buoc
            if buoc == 10 :
                
                self.solanhoatdong = self.solanhoatdong + 1
                isTag  = 0
                for i in xrange(3): # kiem tra may anh co nhin dj Tag hay k 
                    print "i=",i
                    if self.is_marker_pose_received == True : 
                        isTag = isTag + 1
                        self.is_marker_pose_received == False
                    rospy.sleep(0.3)

                if isTag != 0:
                    self.tt_x, self.tt_z, self.tt_g = self.fnGetDataTag(self.MARKER_ID_DETECTION,20) # rad
                    # self.tt_x, self.tt_z, self.tt_g = self.fnGetDataTag_notFilter(self.MARKER_ID_DETECTION) # rad
                    x_robot = (self.tt_x - self.dolech_x_CamVsRObot ) *100 #cm
                    y_robot =  self.tt_z * 100           # cm
                    goc     =  (self.tag_g*180)/np.pi    # do          
                    self.write_log(self.solanhoatdong , self.MARKER_ID_DETECTION , x_robot, y_robot, goc)
                else :
                    self.write_log(self.solanhoatdong , 0 , 99,99,99)

                buoc = -1 # doi parking tiep theo

                print "DONE!!!"
                # rospy.spin()
                # print "tt_g: ",self.tt_g    # quatanion Robot so voi Tag
                # print "tag_g: ",self.tag_g  # vi tri diem tag so voi Robot


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
        j1 = StiAutomaticParking()
        j1.start()

 
        # Keep the main thread running, otherwise signals are ignored.
        while True:
            time.sleep(0.5)
 
    except ServiceExit:
        # Terminate the running threads.
        # Set the shutdown flag on each thread to trigger a clean shutdown of each thread.
        j1.shutdown_flag.set()
        # Wait for the threads to close...
        j1.join()

 
    print('Exiting main program')

if __name__ == '__main__':
    main()