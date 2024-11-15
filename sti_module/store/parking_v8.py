#!/usr/bin/env python

import rospy
import numpy as np
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Twist ,Pose ,Point

import math
import time
import threading
import signal

#global 
MARKER_ID_DETECTION = 0
tt_x = .0
tt_z = .0
tt_g = .0
is_marker_pose_received = False 
is_marker_pose_received2 = False 
tag_g = .0 # robot huong vao apriltag thi tag_g =0

class StiAutomaticParking(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.sub_odom_robot = rospy.Subscriber('/odom', Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_log = rospy.Publisher('/data_log', Pose, queue_size=1)

        self.pub = rospy.Publisher('/parking_client', Pose, queue_size=10) 
        rospy.Subscriber("/parking_server", Pose, self.callSup)

        self.is_odom_received = False  
        rospy.on_shutdown(self.fnShutDown)

        self.rb_x = .0
        self.rb_z = .0
        self.rb_g = .0

        self.odom_x = .0
        self.odom_y = .0
        self.odom_g = .0

        self.enable_parking = False
        self.idtag_parking = 0
        
        self.solanhoatdong = 0
        self.dolech_x_CamVsRObot = -0.027 # m : lech Trai
        self.dolech_TagVsRObot = 0.55 # m
        self.khoangCachDocTag = 1.6
        self.doChinhXac = 0.03 # vi tri phu 50cm

        self.v_robot = False

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
        global MARKER_ID_DETECTION
        # self.idtag_parking = data.position.x
        MARKER_ID_DETECTION = int(data.position.x)
        # print "MARKER_ID_DETECTION : ",MARKER_ID_DETECTION
        if MARKER_ID_DETECTION != 0 :
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
        
        # van toc robot 
        if((math.fabs(robot_odom_msg.twist.twist.linear.x ) < 0.001) and \
           (math.fabs(robot_odom_msg.twist.twist.angular.z ) < 0.001) ) :
            self.v_robot = True
        else :
            self.v_robot = False
        # http://docs.ros.org/jade/api/tf/html/python/transformations.html#references
       
    def fnGetDataTag(self,sl_loc): 
        global tt_x, tt_z, tt_g 
        x_ht = z_ht = g_ht = .0
        x = z = g = .0
        dem = 0
        for dem in range(sl_loc):
            # print tt_x
            while x_ht == tt_x :
                dem = dem
            else:
                # print "dem :" ,dem
                if( sl_loc - dem <= 5):
                    x = x + tt_x 
                    z = z + tt_z 
                    g = g + tt_g 
                    # print "tt_x: ", tt_x
                    
                x_ht = tt_x
                dem = dem + 1
        # lay trung binh 5 lan cuoi
        x = x / 5
        z = z / 5
        g = g / 5
        return x, z, g

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist()) 

    def fnCalcDistPoints(self, x1, x2, y1, y2):
        print "fnCalcDistPoints"
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def run(self):
        # global MARKER_ID_DETECTION
        loop_rate = rospy.Rate(10) # 10hz
        buoc = -1
        # odom_x_ht = self.odom_x
        # odom_y_ht = self.odom_y
        while not self.shutdown_flag.is_set():
            self.park_info(buoc) 
          #Test          
            if buoc == 0 :
                self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag(20)
                # print "odom_g = ", self.odom_g
                # self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag_notFilter(self.MARKER_ID_DETECTION) # rad
                # x_target_phu = self.rb_x - self.dolech_x_CamVsRObot
                # print "x_target_phu: ", x_target_phu
                print "tt_x= %f , tt_z= %f, tt_g= %f" %(self.rb_x, self.rb_z, self.rb_g)
                # s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                # print "s: ", s
                # print "tag_g: ", tag_g
                rospy.spin()

          # Nhan thong tin tu server
            if buoc == -1 :
                if self.enable_parking == True:
                    # self.MARKER_ID_DETECTION = MARKER_ID_DETECTION #self.idtag_parking
                    self.enable_parking = False
                    buoc = 1 
                    print "buoc : ", buoc
                    print "MARKER_ID_DETECTION, " , MARKER_ID_DETECTION

          #B1: Huong camera ve Apriltag          
            if buoc == 1 :
                # rospy.sleep(1)
                print "tag_g: ", tag_g
                twist = Twist()
                while tag_g ==0 : buoc = buoc 
                if math.fabs(tag_g) > 0.1 : #2do
                    print "a"
                    if tag_g > 0 : twist.angular.z = -0.15
                    else :         twist.angular.z = 0.15
                    self.pub_cmd_vel.publish(twist)
                else:
                    print "bb"
                    print "self.v_robot :" , self.v_robot 
                    self.pub_cmd_vel.publish(Twist())
                    if self.v_robot == True:
                        buoc = 2 
                        print "buoc : ", buoc
                        # rospy.spin()

          #B2: Tinh goc can quay, tt_x , tt_z
            
            if buoc == 2 : 
                print "MARKER_ID_DETECTION, " , MARKER_ID_DETECTION
                self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag(20) # rad
                print "tt_x= %f , tt_z= %f, tt_g= %f" %(self.rb_x, self.rb_z, self.rb_g)
                print "odom_g = ", self.odom_g
              
              # tinh x can di
                x_target = self.rb_x - self.dolech_x_CamVsRObot 
                #--> self.rb_x la toa do robot voi tag
                #--> x_target: quang dg robot can di de cam thang tag

                if math.fabs(x_target) <= self.doChinhXac :
                    odom_x_ht = self.odom_x
                    odom_y_ht = self.odom_y
                    buoc = 6
                    print "buoc : ", buoc
                    # rospy.spin()
                else :
                  # tinh goc quay
                    quayTrai = quayPhai = False
                    goc_canquay = math.fabs(self.rb_g)
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
                    print "buoc : ", buoc
                    # rospy.spin()

          #B3: Quay vuong goc           
            if buoc == 3 :
                print "odom_g = ", self.odom_g
                print "goc_taget= " , goc_taget
                print "x_target: ", x_target
                twist = Twist()

                if(goc_taget > 0):
                    print "a"
                    if math.fabs(self.odom_g - goc_taget) > 0.04 :
                        print "b"
                        if x_target < 0 : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
                        else :             twist.angular.z = math.fabs(self.odom_g - goc_taget)
                        if twist.angular.z > 0.7 : twist.angular.z = 0.7
                        if twist.angular.z < -0.7 : twist.angular.z = -0.7
                        self.pub_cmd_vel.publish(twist)
                    else:
                        print "c"
                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        if self.v_robot == True:
                            buoc = 4
                            print "buoc : ", buoc
                            # rospy.spin()
                else:
                    print "m"
                    if math.fabs(self.odom_g - goc_taget) > 0.04 :
                        if x_target < 0 : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
                        else :            twist.angular.z = math.fabs(self.odom_g - goc_taget)
                        if twist.angular.z > 0.7 : twist.angular.z = 0.7
                        if twist.angular.z < -0.7 : twist.angular.z = -0.7
                        self.pub_cmd_vel.publish(twist)
                    else:
                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        if self.v_robot == True:
                            buoc = 4
                            print "buoc : ", buoc
                            # rospy.spin() 

          #B4: Di chuyen chinh giua Apriltag         
            if buoc == 4 :
                twist = Twist()
                s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                print "x_target: ", x_target
                print "s: ", s
                if math.fabs(s) < (math.fabs(x_target)) : 
                    # twist.linear.x = 0.05
                    twist.linear.x = + ( math.fabs(x_target) - math.fabs(s) ) + 0.02
                    if twist.linear.x > 0.4 : twist.linear.x = 0.4
                    self.pub_cmd_vel.publish(twist)
                    
                else : 
                    self.pub_cmd_vel.publish(Twist())
                    buoc = 55
                    print "buoc : ", buoc
                    # rospy.spin() 

          #B55: tinh goc quay lai
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
                print "buoc : ", buoc
                # rospy.spin()

          #B5: quay nguoc lai     
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
                        if quayPhai == True : twist.angular.z = math.fabs(self.odom_g - goc_taget)
                        if quayTrai == True : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
                        if twist.angular.z > 0.7 : twist.angular.z = 0.7
                        if twist.angular.z < -0.7 : twist.angular.z = -0.7
                        self.pub_cmd_vel.publish(twist)
                    else:
                        print "c"
                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        buoc = 6
                        print "buoc : ", buoc 
                        # rospy.spin()

                else:
                    print "m"
                    if math.fabs(self.odom_g - goc_taget) > 0.03 :
                        if quayPhai == True : twist.angular.z = math.fabs(self.odom_g - goc_taget)
                        if quayTrai == True : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
                        if twist.angular.z > 0.7 : twist.angular.z = 0.7
                        if twist.angular.z < -0.7 : twist.angular.z = -0.7
                        self.pub_cmd_vel.publish(twist)
                    else:
                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        buoc = 6
                        print "buoc : ", buoc
                        # rospy.spin()
            
          #B6: tinh chinh goc         
            if buoc == 6 : 
                # rospy.sleep(1)
                print "tag_g: ", tag_g
                print "x_target: ", x_target
                # print "s: ", s
                twist = Twist()
                # if math.fabs(tag_g) > 0.020 : #2do
                #     if tag_g > 0 : twist.angular.z = -0.1
                #     else :              twist.angular.z = 0.1
                #     self.pub_cmd_vel.publish(twist)

                # elif math.fabs(tag_g) <= 0.020 and math.fabs(tag_g) > 0.010 : #2do
                #     if tag_g > 0 : twist.angular.z = -0.06
                #     else :              twist.angular.z = 0.06
                #     self.pub_cmd_vel.publish(twist)

                if math.fabs(tag_g) > 0.05 : #2do
                    print "a"
                    if tag_g > 0 : twist.angular.z = -0.06
                    else :         twist.angular.z = 0.06
                    self.pub_cmd_vel.publish(twist)

                else:
                    self.pub_cmd_vel.publish(Twist())
                    if self.v_robot == True:
                        buoc = 7
                        print "buoc : ", buoc
                    # rospy.spin()

          #B7: tinh kc di vao       
            if buoc == 7 : 
                self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag(20) # rad
                print "tt_x= %f , tt_z= %f, tt_g= %f" %(self.rb_x, self.rb_z, self.rb_g)

                z_target = self.rb_z
                print "z_target: ", z_target
                # tinh x can di
                x_target = self.rb_x - self.dolech_x_CamVsRObot
             
                if math.fabs(x_target) > self.doChinhXac : # dung sai +- 2cm
                    buoc = 77
                else : 
                    buoc = 8
                # rospy.spin()
          
                    #B2: Tinh goc can quay, tt_x , tt_z
           
           #B77 : Tinh laij buoc 2
            
            if buoc == 77 : 
                print "tt_x= %f , tt_z= %f, tt_g= %f" %(self.rb_x, self.rb_z, self.rb_g)
                print "odom_g = ", self.odom_g
              
                # tinh goc quay
                quayTrai = quayPhai = False
                goc_canquay = math.fabs(self.rb_g)
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
      
                else : # quay phai ( + giam , - tang)
                    print "quay phai"
                    quayPhai = True
                    if self.odom_g > 0 : 
                        goc_taget= self.odom_g - goc_canquay
                    else : 
                        goc_taget= self.odom_g - goc_canquay
                        if goc_taget < -np.pi :
                            goc_taget = goc_taget + 2*np.pi

                print "goc_taget= " , goc_taget
                print "x_target: ", x_target

                buoc = 3
                # rospy.spin()
                
          #B8: tinh tien di vao      
            if buoc == 8 :
                global tt_x, tt_z, tt_g 
                print "tt_x= %f , tt_z= %f, tt_g= %f" %(tt_x, tt_z, tt_g )
                twist = Twist()
                s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                x_target_phu = tt_x - self.dolech_x_CamVsRObot
                print "z_target: ", z_target
                print "x_target_phu: ", x_target_phu

                print "s: ", s
                print "tag_g: ", tag_g
                if math.fabs(s) < (math.fabs(z_target)-self.dolech_TagVsRObot) : 
                    if math.fabs(s) < 0.5:
                        twist.linear.x = 0.15
                        if tag_g > 0.015  : 
                            twist.angular.z = - (math.fabs(x_target_phu)) 
                        if tag_g < -0.015 : 
                            twist.angular.z =    math.fabs(x_target_phu)
                    else:
                        twist.linear.x = 0.1
                        if tag_g > 0.015  : 
                            twist.angular.z = - ( math.fabs(x_target_phu)) 
                        if tag_g < -0.015 : 
                            twist.angular.z =  ( math.fabs(x_target_phu)) 

                    if twist.angular.z > 0.3 : twist.angular.z = 0.3
                    if twist.angular.z < -0.3 : twist.angular.z = -0.3

                    self.pub_cmd_vel.publish(twist)
                    
                else : 
                    self.pub_cmd_vel.publish(Twist())
                    buoc = 10
                    # rospy.spin() 

          #B10: publish Log       
            if buoc == 10 :
                global is_marker_pose_received2 
                self.solanhoatdong = self.solanhoatdong + 1
                isTag  = 0
                for i in xrange(3): # kiem tra may anh co nhin dj Tag hay k 
                    print "i=",i
                    if is_marker_pose_received2 == True : 
                        isTag = isTag + 1
                        is_marker_pose_received2 == False
                    rospy.sleep(0.3)

                if isTag != 0:
                    self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag(20) # rad
                    # self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag_notFilter(self.MARKER_ID_DETECTION) # rad
                    x_robot = (self.rb_x - self.dolech_x_CamVsRObot ) *100 #cm
                    y_robot =  self.rb_z * 100           # cm
                    goc     =  (tag_g*180)/np.pi    # do          
                    self.write_log(self.solanhoatdong , MARKER_ID_DETECTION , x_robot, y_robot, goc)

                    print "saiso_x (cm)=", x_robot
                else :
                    self.write_log(self.solanhoatdong , 0 , 99,99,99)
                    print "no tag!!"

                buoc = -1 # doi parking tiep theo
                self.park_info(buoc) 

                print "DONE!!!"
                # rospy.spin()
                # print "tt_g: ",self.rb_g    # quatanion Robot so voi Tag
                # print "tag_g: ",tag_g  # vi tri diem tag so voi Robot


            time.sleep(0.01)
        print('Thread 1 #%s stopped' % self.ident)

class ReadApriltag(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()

        # rospy.init_node('tag_detections_filter')

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.sub_info_marker = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.cbGetMarkerOdom, queue_size = 1)
        self.pub_filter = rospy.Publisher('/tag_detections_filter', Point, queue_size=1)

        self.id_tag = 0
        self.k1 = self.x_ht1 = self.x_truoc1 = 0.
        self.f1 = [0.1, 1., 0.5] # r,p,q
        self.k2 = self.x_ht2 = self.x_truoc2 = 0.
        self.f2 = [0.1, 1., 0.5] # r,p,q
        self.k3 = self.x_ht3 = self.x_truoc3 = 0.
        self.f3 = [0.1, 1., 0.5] # r,p,q
   
        # void thongso(float _r, float _p, float _q){
        #     r=_r;
        #     p=_p;
        #     q=_q;  
        # }
        # double loc(double input){  
        #     k=p/(p+r);
        #     x_ht=x_truoc + k*(input-x_truoc);
        #     p=(1-k)*p+ fabs(x_truoc-x_ht)*q;
        #     x_truoc=x_ht;
        #     return x_ht;
        # }
        # plot: rqt_plot //tag_detections_filter/x:y

    def my_kalman1(self,input):
        self.k1 = self.f1[1]/(self.f1[1] + self.f1[0])
        self.x_ht1 = self.x_truoc1 + self.k1*(input-self.x_truoc1)
        self.f1[1] = (1-self.k1)*self.f1[1]+ math.fabs(self.x_truoc1-self.x_ht1)*self.f1[2]
        self.x_truoc1 = self.x_ht1    
        return self.x_ht1

    def my_kalman2(self,input):
        self.k2 = self.f2[1]/(self.f2[1] + self.f2[0])
        self.x_ht2 = self.x_truoc2 + self.k2*(input-self.x_truoc2)
        self.f2[1] = (1-self.k2)*self.f2[1]+ math.fabs(self.x_truoc2-self.x_ht2)*self.f2[2]
        self.x_truoc2 = self.x_ht2      
        return self.x_ht2

    def my_kalman3(self,input):
        self.k3 = self.f3[1]/(self.f3[1] + self.f3[0])
        self.x_ht3 = self.x_truoc3 + self.k3*(input-self.x_truoc3)
        self.f3[1] = (1-self.k3)*self.f3[1]+ math.fabs(self.x_truoc3-self.x_ht3)*self.f3[2]
        self.x_truoc3 = self.x_ht3    
        return self.x_ht3

    def cbGetMarkerOdom(self, markers_odom_msg): # 5Hz
        global is_marker_pose_received , tag_g , is_marker_pose_received2
        sl_tag = len(markers_odom_msg.detections)
        for sl_tag in range(sl_tag):            
            # if markers_odom_msg.detections[sl_tag-1].id[0] == MARKER_ID_DETECTION:
            if is_marker_pose_received == False:
                is_marker_pose_received = True
            if is_marker_pose_received2 == False:
                is_marker_pose_received2 = True
    
            marker_odom = markers_odom_msg.detections[sl_tag-1]

            tag_g = math.atan2(marker_odom.pose.pose.pose.position.x, \
                                marker_odom.pose.pose.pose.position.z)

    def fnFilterDataTag(self,id_tag): 
        global is_marker_pose_received
        global tt_x, tt_z, tt_g 
        name_tag = "tag_" + str(int(id_tag))
        if is_marker_pose_received == True :
            self.tf_listener.waitForTransform(name_tag,"base_footprint",rospy.Time(),rospy.Duration(1))
            position, quaternion = self.tf_listener.lookupTransform(name_tag, "base_footprint", rospy.Time())
            quaternion = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]

            tt_x = self.my_kalman1(position[0])
            # tt_z = position[0] # t
            tt_z = self.my_kalman2(position[2])
            tt_g = self.my_kalman3(theta) # radian
            is_marker_pose_received = False
            # print "tt_x= %f , tt_z= %f, tt_g= %f" %(tt_x, tt_z, tt_g)
            # print "tt_x=",position[0]

    def run(self):
        global MARKER_ID_DETECTION, tt_x, tt_z, tt_g 
        data_filter = Point()
        while not self.shutdown_flag.is_set():

            if MARKER_ID_DETECTION != 0 :
                self.fnFilterDataTag(MARKER_ID_DETECTION)

                # plot : rqt_plot /tag_detections_filter/
                data_filter.x = tt_x
                data_filter.y = tt_z
                data_filter.z = tt_g
                # print "tt_x= %f , tt_z= %f, tt_g= %f" %(tt_x, tt_z, tt_g)
                self.pub_filter.publish(data_filter)
            # else : print " MARKER_ID_DETECTION =", MARKER_ID_DETECTION
            
            time.sleep(0.01)
        print('Thread 2#%s stopped' % self.ident)


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
 
    rospy.init_node('sti_automatic_parking')
    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:
        j1 = ReadApriltag(1)
        j2 = StiAutomaticParking(2)
        j1.start()
        j2.start()
        

 
        # Keep the main thread running, otherwise signals are ignored.
        while True:
            time.sleep(0.5)
 
    except ServiceExit:
        # Terminate the running threads.
        # Set the shutdown flag on each thread to trigger a clean shutdown of each thread.
        j1.shutdown_flag.set()
        j2.shutdown_flag.set()
        # Wait for the threads to close...
        j1.join()
        j2.join()

 
    print('Exiting main program')

if __name__ == '__main__':
    main()