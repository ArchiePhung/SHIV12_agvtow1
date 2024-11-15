#!/usr/bin/env python
# Author : AnhTuan 25/8/2020
# update : 8/11/2020  : quay de tim tag khi k nhinf thay tag luc ban dau ( rotate if no find Tag when start)
# update : 21/11/2020 : di theo linetu khi dan do bat duoc line ( doc line tu qua rs232)

import rospy
import numpy as np
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Twist ,Pose ,Point
from sti_msgs.msg import *
import math
import time
import threading
import signal

#global 
idTag_target = 0
tolerance = 3
distance_target = 0
tt_x = .0 # kc Tag <-> tam robot
tt_z = .0
tt_g = .0
is_marker_pose_received = False 
is_marker_pose_received2 = False 
tag_g = .0 # robot huong vao apriltag thi tag_g =0
find_id = 0 # if robot dang nhin thay
id_legal = False

# odom_x_ht = 0.
# odom_y_ht = 0.

class StiAutomaticParking(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()

        #get param 
        self.dolech_x_CamVsRObot = rospy.get_param('~dolech_x_CamVsRObot',0)
        self.tolerance_max = rospy.get_param('~tolerance_max',0.05)
        self.tolerance_min = rospy.get_param('~tolerance_min',0.02)
        self.distance_max  = rospy.get_param('~distance_max',1)
        self.distance_min  = rospy.get_param('~distance_min',0.02)
        self.x_robot       = rospy.get_param('~x_robot',1)

        self.rate = rospy.Rate(100)
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.sub_odom_robot = rospy.Subscriber('/odom', Odometry, self.cbGetRobotOdom, queue_size = 100)
        self.sub_zone       = rospy.Subscriber('/zone_robot',Zone_lidar_2head, self.cbZone, queue_size = 100)
        self.pub_cmd_vel    = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.pub_log        = rospy.Publisher('/data_log', Pose, queue_size=100)

        self.pub_status = rospy.Publisher('/parking_status', Parking_status, queue_size=100) 
        rospy.Subscriber("/parking_control", Parking_control, self.callSup)

        rospy.Subscriber("/main_status", Main_status, self.callSup1)
        rospy.Subscriber("/magneticLine1", Magnetic_line, self.callSup2)
        rospy.Subscriber("/magneticLine2", Magnetic_line, self.callSup3)

        self.is_odom_received = False  
        rospy.on_shutdown(self.fnShutDown)

        self.rb_x = .0
        self.rb_z = .0 
        self.rb_g = .0

        self.odom_x = .0
        self.odom_y = .0
        self.odom_g = .0

        self.enable_parking = False
        self.solanhoatdong  = 0
        self.v_robot = False

        self.time_ht = rospy.get_time()
        self.time_tr = rospy.get_time()
        self.rate_cmdvel = 10. 

        # time
        self.t1_ht = rospy.get_time()
        self.t1_tr = rospy.get_time()
        #zone
        self.zone_robot = Zone_lidar_2head()
        # lintu
        self.aBit = 0.
        self.line1 = 0.
        self.line2 = 0.
   
    def callSup1(self,data):
        self.aBit = data.aBit

    def callSup2(self,line1):
        if line1.status == 1 : self.line1 = line1.value
        else : self.line1 = -2 # error magLine


    def callSup3(self,line2):
        if line2.status == 1 : self.line2 = line2.value
        else : self.line2 = -2 # error magLine

    def cbZone(self,data):
        self.zone_robot = data

    def write_log(self,solan,isTag,x,y,g):
        log = Pose()
        log.position.x = solan
        log.position.y = isTag # 0 : k nhin thay Tag , 1 : co Tag

        log.orientation.x = x
        log.orientation.y = y
        log.orientation.z = g
        self.pub_log.publish(log)

    def pub_cmdVel(self, twist , rate , time):
        self.time_ht = time 
        # print self.time_ht - self.time_tr
        # print 1/float(rate)
        if self.time_ht - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = self.time_ht
            self.pub_cmd_vel.publish(twist)
        else :
            pass
            # rospy.loginfo("Hz /cmd_vel OVER !! - %f", 1/float(self.time_ht - self.time_tr) )

        # self.pub_cmd_vel.publish(twist)

    def park_info(self, status,find_tag, ss_x, ss_y, ss_g):
        park = Parking_status()
        park.status  = status # 0: wait , 1-9 : running, 10 : done
        park.find_tag = find_tag
        park.saiso_x = ss_x
        park.saiso_y = ss_y
        park.saiso_g = ss_g
        self.pub_status.publish(park)
    
    def constrain(self,val, min_val, max_val):
        if val < min_val: return min_val
        if val > max_val: return max_val
        return val

    def callSup(self,data):
        global idTag_target,distance_target, tolerance
        idTag_target = data.idTag
        tolerance = self.constrain(data.tolerance, self.tolerance_min , self.tolerance_max)
        distance_target = self.constrain(data.distance, self.distance_min , self.distance_max)
        # rospy.loginfo("distance_target : %s",distance_target )
        # rospy.spin()
        # rospy.loginfo("idTag_target : ",idTag_target )
        if idTag_target > 0 :
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
        if((math.fabs(robot_odom_msg.twist.twist.linear.x ) < 0.01) and \
           (math.fabs(robot_odom_msg.twist.twist.angular.z ) < 0.01) ) :
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
        # print "fnCalcDistPoints"
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)
     
    # def run_lineMag(self):
    #     global odom_x_ht , odom_y_ht ,distance_target, z_target
    #     twist = Twist()
    #     if self.line1 == -1 : 
    #         print("ngoai line ")
    #         return 0 
    #     if self.line1 == 20 :
    #         self.line1 = 7.5 # cho di thang
    #         print("line ngang")
            
    #     if self.line1 > -1 and self.line1 < 16 : 
    #         err = self.line1 - 7.5 
    #         print("err= {}".format(err))
        

    #     s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
    #     z_target = self.rb_z
    #     if math.fabs(s) < (math.fabs(z_target)-distance_target - self.x_robot/2) : 
    #         rospy.logwarn("s= {}".format(s))
    #         kp = 0.05
    #         twist.linear.x = 0.2
    #         twist.angular.z = -1 * err * kp
    #         self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
    #         print("twist= {}".format(twist))
    #     else : 
    #         self.pub_cmd_vel.publish(Twist())
    #         rospy.logwarn("ok : s= {}".format(s))
    #         return 1 


    def run(self):
        global idTag_target,tag_g ,tt_x, tt_z, tt_g , find_id , id_legal , tolerance , distance_target
        # global odom_x_ht , odom_y_ht 
        # loop_rate = rospy.Rate(10) # 10hz
        buoc = -1
        odom_x_ht = 0.
        odom_y_ht = 0.
        while not self.shutdown_flag.is_set():

          # reset
            if (self.enable_parking == False)  and (buoc != -1) :  
                # twist = Twist()
                # self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                for i in range(3):
                    self.pub_cmd_vel.publish(Twist())
                rospy.loginfo("Reset")
                buoc = -1

            if (self.enable_parking == False)  and (buoc == -1) :
                rospy.loginfo("Reset")
                buoc = -1

          # pub status
            self.park_info(buoc,find_id,0,0,0) 
          #Test          
            if buoc == 0 :
                self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag(20)
                # print "odom_g = ", self.odom_g
                # self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag_notFilter(self.idTag_target) # rad
                # x_target_phu = self.rb_x - self.dolech_x_CamVsRObot
                # print "x_target_phu: ", x_target_phu
                # print "tt_x= %s , tt_z= %s, tt_g= %s" %(self.rb_x, self.rb_z, self.rb_g)
                # s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                # print "s: ", s
                # print "tag_g: ", tag_g
                rospy.spin()

          #B0 Wait thong tin tu server
            if buoc == -1 :
                rospy.loginfo("buoc: %s",buoc)
                self.park_info(buoc,find_id,0,0,0) 
                if self.enable_parking == True:
                    self.enable_parking = False
                    rospy.sleep(0.1)
                    if id_legal == True and find_id != 0:
                        buoc = 1 
                        rospy.loginfo("buoc: %s",buoc)
                        rospy.loginfo("idTag_target: %s",idTag_target)

                    elif find_id == 0:
                        buoc = -4 
                        # time
                        self.t1_ht = rospy.get_time()
                        self.t1_tr = rospy.get_time()

          #Buoc -4 : quay trai 5s tim TAG
            if buoc == -4 :
                self.park_info(buoc,find_id,0,0,0)
                twist = Twist()
                self.t1_ht = rospy.get_time()
                if ( id_legal == False ) and (self.t1_ht - self.t1_tr < 5 ):
                    twist.angular.z = 0.2
                    self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                elif ( id_legal == False ) and (self.t1_ht - self.t1_tr > 5 ):
                    self.pub_cmd_vel.publish(Twist())
                    buoc = -5
                    rospy.loginfo("buoc: %s",buoc)
                elif ( id_legal != False ):
                    self.pub_cmd_vel.publish(Twist())
                    buoc = 1

          #Buoc -5 : quay phai tim TAG
            if buoc == -5 :
                rospy.loginfo("quay phai tim TAG")
                self.park_info(buoc,find_id,0,0,0)
                twist = Twist()
                if id_legal == False or find_id == 0 :
                    twist.angular.z = -0.2
                    self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                elif id_legal == True and find_id != 0 : 
                    self.pub_cmd_vel.publish(Twist())
                    buoc  = 1
      
		  #B1: Confirm  da nhin thay Tag hay chua , truong hop tinh goc qua sai 
            if buoc == 1 :
                if find_id == 0 : # k nhin thay Tag
                    buoc = -5 # quay phai tim Tag 
                else : 
                    buoc = 100

          #B11: Huong camera ve Apriltag , NOTE : check yes/no Tag           
            if buoc == 100 :
                self.park_info(buoc,find_id,0,0,0) 
                twist = Twist()
                # while tag_g == 0 : buoc = buoc 

                if find_id == 0 : 
                    twist.angular.z = 0
                    self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    buoc = 100
                else :
                    if math.fabs(tag_g) > 0.1: #2do

                        if tag_g > 0 : twist.angular.z = -0.15
                        else :         twist.angular.z = 0.15
                        self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    else:
                        self.pub_cmd_vel.publish(Twist())
                        if self.v_robot == True:
                            buoc = 2
                            rospy.loginfo("buoc: %s",buoc)
           
          #B2: Tinh goc can quay, tt_x , tt_z
            if buoc == 2 : 
                self.park_info(buoc,find_id,0,0,0) 
                self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag(20) # rad
                rospy.loginfo("idTag_target: %s",idTag_target)
                rospy.loginfo("tt_x= %s , tt_z= %s, tt_g= %s" ,self.rb_x, self.rb_z, self.rb_g)
                rospy.loginfo("odom_g= %s ", self.odom_g)
              
              # tinh x can di
                x_target = self.rb_x - self.dolech_x_CamVsRObot 
                #--> self.rb_x la toa do robot voi tag
                #--> x_target: quang dg robot can di de cam thang tag

                if math.fabs(x_target) <= tolerance :
                    odom_x_ht = self.odom_x
                    odom_y_ht = self.odom_y
                    buoc = 66
                    rospy.loginfo("buoc: %s",buoc)
                    
                else :
                  # tinh goc quay
                    quayTrai = quayPhai = False
                    goc_canquay = math.fabs(self.rb_g)
                    if x_target > 0 : # quay trai ( + tang , - giam)
                        rospy.loginfo("quay trai")
                        quayTrai = True
                        if self.odom_g > 0 : 
                            goc_taget= self.odom_g + goc_canquay
                            if goc_taget > np.pi : 
                                goc_bu = goc_taget - np.pi 
                                goc_taget = goc_bu - np.pi 
                        else : 
                            goc_taget= self.odom_g + goc_canquay

                    else : # quay phai ( + giam , - tang)
                        rospy.loginfo("quay phai")
                        quayPhai = True
                        if self.odom_g > 0 : 
                            goc_taget= self.odom_g - goc_canquay
                        else : 
                            goc_taget= self.odom_g - goc_canquay
                            if goc_taget < -np.pi :
                                goc_taget = goc_taget + 2*np.pi

                    rospy.loginfo("goc_taget= %s " , goc_taget)
                    rospy.loginfo("x_target: %s", x_target)

                    buoc = 31
                    rospy.loginfo("buoc: %s",buoc)
            
          #B111: check line tu 
            if buoc == 31 : 
                self.park_info(buoc,find_id,0,0,0) 
                rospy.loginfo("buoc: %s",buoc)
                # print("self.Line_ahead = %s", self.Line_ahead )
                # math.fabs(x_target) < 0.1 --> truong hop robot dungs lech tag va bat line tu khac
                if self.line1 > -1 and self.line1 < 16 and math.fabs(x_target) < 0.5:
                    buoc = 32  
                    odom_x_ht = self.odom_x
                    odom_y_ht = self.odom_y
                #     self.solanhoatdong = self.solanhoatdong + 1
                #     self.write_log(self.solanhoatdong , find_id , 66,66,66)
                #     buoc = 11
                # else :
                #     buoc = 3

            if buoc == 32 : 
                rospy.loginfo("buoc: %s",buoc)
                self.park_info(buoc,find_id,0,0,0) 
                twist = Twist()
                if self.line1 == -1 : 
                    print("ngoai line ")
                    return 0 
                if self.line1 == 20 :
                    self.line1 = 7.5 # cho di thang
                    print("line ngang")
                    
                if self.line1 > -1 and self.line1 < 16 : 
                    err = self.line1 - 7.5 
                    print("err= {}".format(err))
                

                s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                z_target = self.rb_z

                rospy.logwarn("s= {}".format((math.fabs(z_target)-distance_target - self.x_robot/2) ))


                if math.fabs(s) < (math.fabs(z_target)-distance_target - self.x_robot/2) : 
                    rospy.logwarn("s= {}".format((math.fabs(z_target)-distance_target - self.x_robot/2) ))
                    kp = 0.05
                    twist.linear.x = 0.2
                    twist.angular.z = -1 * err * kp
                    self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    print("twist= {}".format(twist))
                else : 
                    self.pub_cmd_vel.publish(Twist())
                    rospy.logwarn("ok : s= {}".format(s))
                    return 1

          #B3: Quay vuong goc           
            if buoc == 3 :
                self.park_info(buoc,find_id,0,0,0) 
                # rospy.loginfo("odom_g = %s", self.odom_g)
                # rospy.loginfo("goc_taget= %s" , goc_taget)
                # rospy.loginfo("x_target: %s", x_target)
                twist = Twist()

                if(goc_taget > 0):
                    rospy.loginfo("A3 :odom_g= %s , goc_taget= %s ", self.odom_g,goc_taget)
                    if math.fabs(self.odom_g - goc_taget) > 0.04 :
                        if x_target < 0 : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
                        else :             twist.angular.z = math.fabs(self.odom_g - goc_taget)
                        if twist.angular.z > 0.4 : twist.angular.z = 0.4
                        if twist.angular.z < -0.4 : twist.angular.z = -0.4
                        self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    else:

                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        if self.v_robot == True:
                            buoc = 4
                            rospy.loginfo("buoc: %s",buoc)
                else:
					rospy.loginfo("B3 :odom_g= %s , goc_taget= %s ", self.odom_g,goc_taget)

					if math.fabs(self.odom_g - goc_taget) > 0.04 :
						if x_target < 0 : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
						else :            twist.angular.z = math.fabs(self.odom_g - goc_taget)
						if twist.angular.z > 0.4 : twist.angular.z = 0.4
						if twist.angular.z < -0.4 : twist.angular.z = -0.4
						self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
					else:
						self.pub_cmd_vel.publish(Twist())
						odom_x_ht = self.odom_x
						odom_y_ht = self.odom_y
						if self.v_robot == True:
							buoc = 4
							rospy.loginfo("buoc: %s",buoc)

          #B4: Di chuyen chinh giua Apriltag         
            if buoc == 4 :
                self.park_info(buoc,find_id,0,0,0) 
                twist = Twist()
                s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                rospy.loginfo("x_target: %s ", x_target)
                rospy.loginfo("s: %s", s )
                if math.fabs(s) < (math.fabs(x_target)) : 
                    twist.linear.x = + ( math.fabs(x_target) - math.fabs(s) ) + 0.02
                    if twist.linear.x > 0.4 : twist.linear.x = 0.4
                    self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    
                else : 
                    self.pub_cmd_vel.publish(Twist())
                    buoc = 5
                    rospy.loginfo("buoc: %s",buoc)

          #B5: tinh goc quay lai
            if buoc == 5 :
                self.park_info(buoc,find_id,0,0,0) 
                rospy.loginfo("odom_g   %s= ", self.odom_g)
                rospy.loginfo("x_target: %s", x_target)
                if quayTrai == True:
                    rospy.loginfo("quay phai")
                    if self.odom_g > 0 : 
                        goc_taget = self.odom_g - np.pi/2
                    else : 
                        goc_taget = self.odom_g - np.pi/2
                        if goc_taget < -np.pi :
                            goc_taget = goc_taget + 2*np.pi


                if quayPhai == True:    
                    rospy.loginfo("quay trai")   
                    if self.odom_g > 0 : 
                        goc_taget= self.odom_g + np.pi/2
                        if goc_taget > np.pi : 
                            goc_bu = goc_taget - np.pi 
                            goc_taget = goc_bu - np.pi 
                    else : 
                        goc_taget= self.odom_g + np.pi/2

                rospy.loginfo("goc_taget_quaylai= %s" , goc_taget  )
                buoc = 6
                rospy.loginfo("buoc : %s", buoc)

          #B6: quay nguoc lai     
            if buoc == 6 :
                self.park_info(buoc,find_id,0,0,0) 
                # rospy.loginfo("odom_g   %s= ", self.odom_g)
                # rospy.loginfo("goc_taget= %s" , goc_taget)
                # rospy.loginfo("x_target: %s", x_target)
                # rospy.loginfo("s: %s", s)
                twist = Twist()
                if(goc_taget > 0):
                    rospy.loginfo("A6 :odom_g= %s , goc_taget= %s ", self.odom_g,goc_taget)
                    if math.fabs(self.odom_g - goc_taget) > 0.04 :
                        if quayPhai == True : twist.angular.z = math.fabs(self.odom_g - goc_taget)
                        if quayTrai == True : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
                        if twist.angular.z > 0.4 : twist.angular.z = 0.4
                        if twist.angular.z < -0.4 : twist.angular.z = -0.4
                        self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    else:
                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        buoc = 66 # 7 

                else:
                    rospy.loginfo("B6 :odom_g= %s , goc_taget= %s ", self.odom_g,goc_taget)
                    if math.fabs(self.odom_g - goc_taget) > 0.04 :
                        if quayPhai == True : twist.angular.z = math.fabs(self.odom_g - goc_taget)
                        if quayTrai == True : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
                        if twist.angular.z > 0.4 : twist.angular.z = 0.4
                        if twist.angular.z < -0.4 : twist.angular.z = -0.4
                        self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    else:
                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        buoc = 66 # 7 
                        rospy.loginfo("buoc : %s", buoc)

		  #B66: Confirm khi quay lai da nhin thay Tag hay chua , truong hop tinh goc qua sai 
            if buoc == 66 :
                if find_id == 0 : # k nhin thay Tag
                    buoc = -5 # quay phai tim Tag 
                else : 
                    buoc = 7 
			
          #B7: tinh chinh goc  , NOTE : check yes/no Tag      
            if buoc == 7 : 
                self.park_info(buoc,find_id,0,0,0) 
                rospy.loginfo("tag_g: %s", tag_g)
                rospy.loginfo("x_target: %s", x_target)
                twist = Twist()

                if find_id == 0 : # --> TH : quay sai , k nhin thay Tah --> dung yen --> CAN KHAC PHUC CHO NAY 
                    twist.angular.z = 0
                    self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    buoc = 7
                else :
                    if math.fabs(tag_g) > 0.05 : #2do
                        if tag_g > 0 : twist.angular.z = -0.06
                        else :         twist.angular.z = 0.06
                        self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())

                    else:
                        self.pub_cmd_vel.publish(Twist())
                        if self.v_robot == True:
                            buoc = 77
                            rospy.loginfo("buoc : %s", buoc)
          
          #B777: check line tu 
            if buoc == 77 : 
                rospy.loginfo("buoc: %s",buoc)
                self.park_info(buoc,find_id,0,0,0) 
                print("self.line1 = %s", self.line1 )
                if self.line1 > -1 and self.line1 < 16 : 
                    buoc = 78
                    odom_x_ht = self.odom_x
                    odom_y_ht = self.odom_y
                #     self.solanhoatdong = self.solanhoatdong + 1
                #     self.write_log(self.solanhoatdong , find_id , 55,55,55)
                #     buoc = 11
                # else :
                #     buoc = 8

            if buoc == 78 : 
                rospy.loginfo("buoc: %s",buoc)
                self.park_info(buoc,find_id,0,0,0) 
                rospy.loginfo("buoc: %s",buoc)
                self.park_info(buoc,find_id,0,0,0) 
                twist = Twist()
                if self.line1 == -1 : 
                    print("ngoai line ")
                    return 0 
                if self.line1 == 20 :
                    self.line1 = 7.5 # cho di thang
                    print("line ngang")
                    
                if self.line1 > -1 and self.line1 < 16 : 
                    err = self.line1 - 7.5 
                    print("err= {}".format(err))
                

                s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                z_target = self.rb_z
                rospy.logwarn("ok : s= {}".format(s))
                rospy.logwarn("ok : z_target= {}".format(z_target))
                rospy.logwarn("s1= {}".format((math.fabs(z_target)-distance_target - self.x_robot/2) ))


                if math.fabs(s) < (math.fabs(z_target)-distance_target - self.x_robot/2) : 
                    
                    kp = 0.05
                    twist.linear.x = 0.2
                    twist.angular.z = -1 * err * kp
                    self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    print("twist= {}".format(twist))
                else : 
                    self.pub_cmd_vel.publish(Twist())
                    # rospy.logwarn("ok : s= {}".format(s))
                    return 1

          #B8: tinh kc di vao       
            if buoc == 8 : 
                self.park_info(buoc,find_id,0,0,0) 
                self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag(20) # rad
                z_target = self.rb_z
                rospy.loginfo("tt_x= %s , tt_z= %s, tt_g= %s",self.rb_x, self.rb_z, self.rb_g)
                rospy.loginfo("z_target: %s", z_target)

                # tinh x can di
                x_target = self.rb_x - self.dolech_x_CamVsRObot
             
                if math.fabs(x_target) > tolerance : # dung sai +- 2cm
                    buoc = 77
                else : 
                    odom_x_ht = self.odom_x
                    odom_y_ht = self.odom_y
                    buoc = 9
           
          #B77 : Tinh lai buoc 2 
            if buoc == 77 : 
                rospy.loginfo("tt_x= %s , tt_z= %s, tt_g= %s",self.rb_x, self.rb_z, self.rb_g)
                rospy.loginfo("odom_g = %s", self.odom_g )
              
                # tinh goc quay
                quayTrai = quayPhai = False
                goc_canquay = math.fabs(self.rb_g)
                if x_target > 0 : # quay trai ( + tang , - giam)
                    rospy.loginfo("quay trai")
                    quayTrai = True
                    if self.odom_g > 0 : 
                        goc_taget= self.odom_g + goc_canquay
                        if goc_taget > np.pi : 
                            goc_bu = goc_taget - np.pi 
                            goc_taget = goc_bu - np.pi 
                    else : 
                        goc_taget= self.odom_g + goc_canquay
      
                else : # quay phai ( + giam , - tang)
                    rospy.loginfo("quay phai")
                    quayPhai = True
                    if self.odom_g > 0 : 
                        goc_taget= self.odom_g - goc_canquay
                    else : 
                        goc_taget= self.odom_g - goc_canquay
                        if goc_taget < -np.pi :
                            goc_taget = goc_taget + 2*np.pi

                rospy.loginfo("goc_taget= %s" , goc_taget)
                rospy.loginfo("x_target: %s", x_target)

                buoc = 3
                # rospy.spin()
                
          #B9: tinh tien di vao  , NOTE : check yes/no Tag      
            if buoc == 9 :
                self.park_info(buoc,find_id,0,0,0) 
                rospy.loginfo("tt_x= %s , tt_z= %s, tt_g= %s",self.rb_x, self.rb_z, self.rb_g)
                twist = Twist()
                s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                x_target_phu = tt_x - self.dolech_x_CamVsRObot

                rospy.loginfo("z_target: %s", z_target)
                rospy.loginfo("x_target_phu: %s", x_target_phu)
                rospy.loginfo("s: %s", s)
                rospy.loginfo("tag_g: %s", tag_g)

                if find_id == 0 : 
                    twist.angular.z = 0
                    self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    buoc = 9
                else :
                    if math.fabs(s) < (math.fabs(z_target)-distance_target - self.x_robot/2) : 
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

                        self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                        
                    else : 
                        self.pub_cmd_vel.publish(Twist())
                        buoc = 10
                        rospy.loginfo("buoc : %s", buoc)

          #B10: publish Log       
            if buoc == 10 :
                global is_marker_pose_received2 
                self.solanhoatdong = self.solanhoatdong + 1
                isTag  = 0
                for i in xrange(3): # kiem tra camera co nhin thay Tag hay khong 
                    # print "i=",i
                    if is_marker_pose_received2 == True : 
                        isTag = isTag + 1
                        is_marker_pose_received2 == False
                    rospy.sleep(0.1)

                if isTag != 0:
                    self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag(20) # rad
                    # self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag_notFilter(self.MARKER_ID_DETECTION) # rad
                    x_robot = (self.rb_x - self.dolech_x_CamVsRObot ) *100 #cm
                    y_robot =  self.rb_z * 100           # cm
                    goc     =  (tag_g*180)/np.pi    # do          
                    self.write_log(self.solanhoatdong , find_id , x_robot, y_robot, goc)
                    # print "saiso_x (cm)=", x_robot
                else :
                    self.write_log(self.solanhoatdong , 0 , 99,99,99)
                    # print "no tag!!"

                buoc = 11

          #B11: doi reset
            if buoc == 11 :
                # self.park_info(buoc,find_id,x_robot, y_robot, goc)
                rospy.loginfo("DONE!!!")
                if idTag_target == 0 :  
                    rospy.loginfo("Reset")
                    buoc = -1

            self.rate.sleep()
        print('Thread 1 #%s stopped' % self.ident)

class ReadApriltag(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()

        #get param 
        self.kalman_r = rospy.get_param('~kalman_r',0.1)
        self.kalman_p = rospy.get_param('~kalman_p',1.0)
        self.kalman_q = rospy.get_param('~kalman_q',0.5)

        # rospy.init_node('tag_detections_filter')
        self.rate = rospy.Rate(100)
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.sub_info_marker = rospy.Subscriber('/tag_detections_d435', AprilTagDetectionArray, self.cbGetMarkerOdom, queue_size = 1)
        self.pub_filter = rospy.Publisher('/tag_detections_filter', Point, queue_size=1)

        # kalman
        self.k1 = self.x_ht1 = self.x_truoc1 = 0.
        self.f1 = [self.kalman_r, self.kalman_p, self.kalman_q] # r,p,q
        self.k2 = self.x_ht2 = self.x_truoc2 = 0.
        self.f2 = [self.kalman_r, self.kalman_p, self.kalman_q] # r,p,q
        self.k3 = self.x_ht3 = self.x_truoc3 = 0.
        self.f3 = [self.kalman_r, self.kalman_p, self.kalman_q] # r,p,q

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
        global is_marker_pose_received , tag_g , is_marker_pose_received2 , find_id
        sl_tag = len(markers_odom_msg.detections)
        if sl_tag == 0 : 
            find_id = 0 
            tag_g = 0 
        else :
            for sl_tag in range(sl_tag):            
                # check id
                find_id = markers_odom_msg.detections[sl_tag-1].id[0] 

                if is_marker_pose_received == False:
                    is_marker_pose_received = True
                if is_marker_pose_received2 == False:
                    is_marker_pose_received2 = True
        
                marker_odom = markers_odom_msg.detections[sl_tag-1]

                tag_g = math.atan2(marker_odom.pose.pose.pose.position.x, \
                                    marker_odom.pose.pose.pose.position.z)

    def fnFilterDataTag(self,id_tag): 
        global is_marker_pose_received , id_legal, find_id
        global tt_x, tt_z, tt_g 
        name_tag = "tag_" + str(int(id_tag))
        if is_marker_pose_received == True :
            if find_id == id_tag :
                id_legal = True
                self.tf_listener.waitForTransform(name_tag,"base_footprint",rospy.Time(),rospy.Duration(1)) 
                position, quaternion = self.tf_listener.lookupTransform(name_tag, "base_footprint", rospy.Time())
                quaternion = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
                theta = tf.transformations.euler_from_quaternion(quaternion)[1]

                tt_x = self.my_kalman1(position[0])
                # tt_z = position[0] # 
                tt_z = self.my_kalman2(position[2])
                tt_g = self.my_kalman3(theta) # radian
                is_marker_pose_received = False
                # print "tt_x= %s , tt_z= %s, tt_g= %s" %(tt_x, tt_z, tt_g)
                # print "tt_x=",position[0]
            else :
                id_legal = False

    def run(self):
        global idTag_target, tt_x, tt_z, tt_g 
        data_filter = Point()
        while not self.shutdown_flag.is_set():

            if idTag_target != 0 :
                self.fnFilterDataTag(idTag_target)

                # plot : rqt_plot /tag_detections_filter/
                data_filter.x = tt_x
                data_filter.y = tt_z
                data_filter.z = tt_g
                # print "tt_x= %s , tt_z= %s, tt_g= %s" %(tt_x, tt_z, tt_g)
                self.pub_filter.publish(data_filter)
            # else : print " idTag_target =", idTag_target
            
            # time.sleep(0.01)
            self.rate.sleep()
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
 
    rospy.init_node('sti_automatic_parking_v13')
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
            time.sleep(0.1)
 
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
