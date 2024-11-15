#!/usr/bin/env python
# Author : AnhTuan 19/8/2020
import rospy
import numpy as np
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Twist ,Pose ,Point
from sti_msgs.msg import Parking_control , Parking_status
import math
import time
import threading
import signal

#global 
idTag_target = 0
tolerance = 2.5
distance_target = 0
tt_x = .0 # kc Tag <-> tam robot
tt_z = .0
tt_g = .0
is_marker_pose_received = False 
is_marker_pose_received2 = False 
tag_g = .0 # robot huong vao apriltag thi tag_g =0
find_id = 0 # if robot dang nhin thay
id_legal = False

cam_d435 = False
cam_phai = False
cam_trai = False

class StiAutomaticParking(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        self.rate = rospy.Rate(100)
        
        self.sub_odom_robot = rospy.Subscriber('/odom', Odometry, self.cbGetRobotOdom, queue_size = 100)
        self.pub_cmd_vel    = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.is_odom_received = False  
        rospy.on_shutdown(self.fnShutDown)

        self.rb_x = .0
        self.rb_z = .0 
        self.rb_g = .0

        self.odom_x = .0
        self.odom_y = .0
        self.odom_g = .0
        self.v_robot = False
        self.time_ht = rospy.get_time()
        self.time_tr = rospy.get_time()
        self.rate_cmdvel = 10.

        self.dolech_x_camD435 = 0.03
        self.dolech_x_camPhai = 0.
        self.dolech_x_CamTrai = 0.
        
    def pub_cmdVel(self, twist , rate , time):
        self.time_ht = time 
        # print self.time_ht - self.time_tr
        # print 1/float(rate)
        if self.time_ht - self.time_tr > float(1/rate) : # < 20hz 
            self.time_tr = self.time_ht
            self.pub_cmd_vel.publish(twist)
        else :
            rospy.logwarn("Hz /cmd_vel OVER !! - %f", 1/float(self.time_ht - self.time_tr) )
        
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

    def checkHz(self,time):
        self.time_ht = time
        hz = 1 / (self.time_ht - self.time_tr)
        self.time_tr = self.time_ht
        return hz
        #use :  print self.checkHz(rospy.get_time())

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist()) 

    def fnCalcDistPoints(self, x1, x2, y1, y2):
        # print "fnCalcDistPoints"
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

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

    def run(self): 
        global tt_x, tt_z, tt_g , tag_g , idTag_target , tolerance
        step = 1
        idTag_target = 11
        while not self.shutdown_flag.is_set():
          # PART 1
            # step 1 : quay chinh giua
            if step == 1 :
                rospy.loginfo("step: %s",step)
                twist = Twist()
                # print "tag_g= %s" %(tag_g)
                while tag_g == 0 : step = step 
                if math.fabs(tag_g) > 0.1 : #2do
                    if tag_g > 0 : twist.angular.z = -0.15
                    else :         twist.angular.z = 0.15
                    self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                else:
                    self.pub_cmd_vel.publish(Twist())
                    if self.v_robot == True:
                        step = 2 
                        rospy.loginfo("step: %s",step)

            # step 2 : tinh goc quay song song , tinh quang duong tien lui can chinh giua   
            if step == 2 :
                rospy.loginfo("step: %s",step)
                self.rb_x, self.rb_z, self.rb_g = self.fnGetDataTag(20) # rad
                rospy.loginfo("idTag_target: %s",idTag_target)
                rospy.loginfo("tt_x= %s , tt_z= %s, tt_g= %s" ,self.rb_x, self.rb_z, self.rb_g)
                rospy.loginfo("odom_g= %s ", self.odom_g)
              
              # tinh x can di
                x_target = self.rb_x - self.dolech_x_CamTrai 
                rospy.loginfo("x_target: %s", x_target)
                
                #--> self.rb_x la toa do robot voi tag
                #--> x_target: quang dg robot can di de cam thang tag

                # if math.fabs(x_target) <= tolerance :
                if 0 : 
                    odom_x_ht = self.odom_x
                    odom_y_ht = self.odom_y
                    step = 7
                    rospy.loginfo("step: %s",step)
                    
                else :
                  # tinh goc quay
                    quayTrai = quayPhai = False
                    goc_canquay = math.fabs(self.rb_g)
                    if x_target < 0 : # quay trai ( + tang , - giam)
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

                    rospy.loginfo("goc_canquay= %s " , goc_canquay)
                    rospy.loginfo("goc_taget= %s " , goc_taget)
                    rospy.loginfo("x_target: %s", x_target)

                    step = 3
                    rospy.loginfo("step: %s",step)
            
            # step 3 : quay song song
            if step == 3 :
                # self.park_info(step,find_id,0,0,0) 
                # rospy.loginfo("odom_g = %s", self.odom_g)
                # rospy.loginfo("goc_taget= %s" , goc_taget)
                # rospy.loginfo("x_target: %s", x_target)
                twist = Twist()

                if(goc_taget < 0):
                    rospy.loginfo("A3 :odom_g= %s , goc_taget= %s ", self.odom_g,goc_taget)
                    if math.fabs(self.odom_g - goc_taget) > 0.04 :
                        if x_target < 0 : twist.angular.z = math.fabs(self.odom_g - goc_taget)
                        else :             twist.angular.z = -math.fabs(self.odom_g - goc_taget)
                        if twist.angular.z > 0.4 : twist.angular.z = 0.4
                        if twist.angular.z < -0.4 : twist.angular.z = -0.4
                        self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    else:

                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        if self.v_robot == True:
                            step = 4
                            rospy.loginfo("step: %s",step)
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
							step = 4
							rospy.loginfo("step: %s",step)
            
            # step 4: Di chuyen chinh giua Apriltag         
            if step == 4 :
                # self.park_info(step,find_id,0,0,0) 
                twist = Twist()
                s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                rospy.loginfo("x_target: %s ", x_target)
                rospy.loginfo("s: %s", s )
                if math.fabs(s) < (math.fabs(x_target)) : 
                    twist.linear.x = + ( math.fabs(x_target) - math.fabs(s) ) + 0.02
                    if twist.linear.x > 0.4 : twist.linear.x = 0.4
                    if x_target > 0 : twist.linear.x = -twist.linear.x # them do can di tien/lui
                    self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    
                else : 
                    self.pub_cmd_vel.publish(Twist())
                    step = 5
                    rospy.loginfo("step: %s",step)

            # step 5 : tinh goc quay lai
            if step == 5 :
                # self.park_info(step,find_id,0,0,0) 
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
                step = 6
                rospy.loginfo("step : %s", step)

          #B6: quay nguoc lai     
            if step == 6 :
                # self.park_info(step,find_id,0,0,0) 
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
                        step = 7

                else:
                    rospy.loginfo("B6 :odom_g= %s , goc_taget= %s ", self.odom_g,goc_taget)
                    if math.fabs(self.odom_g - goc_taget) > 0.04 :
                        if quayPhai == True : twist.angular.z = -math.fabs(self.odom_g - goc_taget)
                        if quayTrai == True : twist.angular.z = math.fabs(self.odom_g - goc_taget)
                        if twist.angular.z > 0.4 : twist.angular.z = 0.4
                        if twist.angular.z < -0.4 : twist.angular.z = -0.4
                        self.pub_cmdVel(twist,self.rate_cmdvel,rospy.get_time())
                    else:
                        self.pub_cmd_vel.publish(Twist())
                        odom_x_ht = self.odom_x
                        odom_y_ht = self.odom_y
                        step = 7
                        rospy.loginfo("step : %s", step)

          # PART 2
            # step 5 : quay chinh giua 
            # step 6 : chack lech trai phai
            # step 7 : tinh tien di vao

            # print self.checkHz(rospy.get_time())

            self.rate.sleep()
        print('Thread #%s stopped' % self.threadID)

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

        self.sub_info_marker = rospy.Subscriber('/tag_detections_d435', AprilTagDetectionArray, self.getTagOdom_d435, queue_size = 30)
        self.sub_info_marker = rospy.Subscriber('/tag_detections_logi_phai', AprilTagDetectionArray, self.getTagOdom_Phai, queue_size = 30)
        self.sub_info_marker = rospy.Subscriber('/tag_detections_logi_trai', AprilTagDetectionArray, self.getTagOdom_Trai, queue_size = 30)
        self.pub_filter      = rospy.Publisher('/tag_detections_filter', Point, queue_size=1)

      # kalman
        self.k1 = self.x_ht1 = self.x_truoc1 = 0.
        self.f1 = [self.kalman_r, self.kalman_p, self.kalman_q] # r,p,q
        self.k2 = self.x_ht2 = self.x_truoc2 = 0.
        self.f2 = [self.kalman_r, self.kalman_p, self.kalman_q] # r,p,q
        self.k3 = self.x_ht3 = self.x_truoc3 = 0.
        self.f3 = [self.kalman_r, self.kalman_p, self.kalman_q] # r,p,q
      # HZ
        self.time_ht = rospy.get_time()
        self.time_tr = rospy.get_time()
  
    def checkHz(self,time):
        self.time_ht = time
        hz = 1 / (self.time_ht - self.time_tr)
        self.time_tr = self.time_ht
        return hz
        #use :  print self.checkHz(rospy.get_time())

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

    def filter_dataTag(self, id_target , odom_in):
      #step 1 : get odom of id_target
        global id_legal,  tt_x, tt_z, tt_g ,tag_g
        sl_tag = len(odom_in.detections)
        if sl_tag != 0 : 
            for sl_tag in range(sl_tag):            
                # check id
                find_id = odom_in.detections[sl_tag-1].id[0] 
                if find_id == id_target :
                    # id_legal = True
                    marker_odom = odom_in.detections[sl_tag-1]

                    tag_g = math.atan2(marker_odom.pose.pose.pose.position.x, \
                                       marker_odom.pose.pose.pose.position.z)
                 #step 2 : filter Klaman
                    name_tag = "tag_" + str(int(id_target))
                    # self.tf_listener.waitForTransform(name_tag,"base_footprint",rospy.Time(),rospy.Duration(0.01))
                    position, quaternion = self.tf_listener.lookupTransform(name_tag, "base_footprint", rospy.Time())
                    quaternion = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
                    theta = tf.transformations.euler_from_quaternion(quaternion)[1]
                    # print "tt_x= %s , tt_z= %s, tt_g= %s" %(position[0], position[2], theta)

                    tt_x = self.my_kalman1(position[0])
                    # tt_z = position[0] # 
                    tt_z = self.my_kalman2(position[2])
                    tt_g = self.my_kalman3(theta) # radian 
                    # print "tt_x= %s , tt_z= %s, tt_g= %s" %(tt_x, tt_z, tt_g)
                    # print "hz= %s \n" %(self.checkHz(rospy.get_time()))
          
    def getTagOdom_d435(self, markers_odom_msg): # 30Hz
        global idTag_target, cam_d435, cam_phai, cam_trai
        self.filter_dataTag(idTag_target,markers_odom_msg) 
        cam_d435 = True
        cam_phai = False
        cam_trai = False

    def getTagOdom_Phai(self, markers_odom_msg): # 30Hz
        global idTag_target, cam_d435, cam_phai, cam_trai
        self.filter_dataTag(idTag_target,markers_odom_msg) 
        cam_d435 = False
        cam_phai = True
        cam_trai = False

    def getTagOdom_Trai(self, markers_odom_msg): # 30Hz
        global idTag_target, cam_d435, cam_phai, cam_trai
        self.filter_dataTag(idTag_target,markers_odom_msg) 
        cam_d435 = False
        cam_phai = False
        cam_trai = True
   
    def run(self):
        global idTag_target
        while not self.shutdown_flag.is_set():
            # idTag_target = 11
         
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
    rospy.init_node('comunication_modbus')
    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:
        thread1 = StiAutomaticParking(1)
        thread1.start()
        thread2 = ReadApriltag(2)
        thread2.start()
 
        # Keep the main thread running, otherwise signals are ignored.
        while True:
            pass
            time.sleep(0.001)
 
    except ServiceExit:
        thread1.shutdown_flag.set()
        thread1.join()
        thread2.shutdown_flag.set()
        thread2.join()
    print('Exiting main program')
 
if __name__ == '__main__':
    main()