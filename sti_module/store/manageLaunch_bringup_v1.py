#!/usr/bin/env python
# author : Anh Tuan - 16/9/2020

from sti_msgs.msg import Velocities, Ps2_msgs, ManageLaunch
# from std_msgs.msg import String
from sensor_msgs.msg import LaserScan , Imu, Image , PointCloud2
from apriltag_ros.msg import AprilTagDetectionArray
from leg_tracker.msg import LegArray
import roslaunch
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

import rospy
import string


class Start_launch:
    def __init__(self):
        rospy.init_node('manageLaunch_bringup', anonymous=True)
        self.rate = rospy.Rate(10)

        #get param 2d_bringup
        self.tf_config_trienlam = rospy.get_param('~tf_config_trienlam','')        
        self.lms100_tim551      = rospy.get_param('~lms100_tim551','')    
        self.detec_leg          = rospy.get_param('~detec_leg','') 
        self.hector_odom_v3     = rospy.get_param('~hector_odom_v3','')  
        self.conver_lidar       = rospy.get_param('~conver_lidar','') 
     
        self.base_8_2           = rospy.get_param('~base_8_2','')        
        self.imu_bno055         = rospy.get_param('~imu_bno055','')        
        self.odom_encoder       = rospy.get_param('~odom_encoder','')                      
        self.ekf                = rospy.get_param('~ekf','')     

        self.D435               = rospy.get_param('~D435','')   
        self.apriltag_d435      = rospy.get_param('~apriltag_d435','')

        # topic test 
        # rospy.Subscriber("/test_node", Float32, self.call_check1)
        # topic lidar
        rospy.Subscriber("/scan_lms100", LaserScan, self.call_check2)
        rospy.Subscriber("/scan_tim551", LaserScan, self.call_check3)
        rospy.Subscriber("/scan", LaserScan, self.call_check4)
        rospy.Subscriber("/detected_leg_clusters", LegArray, self.call_check5) 
        rospy.Subscriber("/scanmatch_odom", Odometry, self.call_check6)
        rospy.Subscriber("/pose_lidar", PoseWithCovarianceStamped, self.call_check7)
        
        # topic encoder + imu + ps2
        rospy.Subscriber("/raw_vel", Velocities, self.call_check8) 
        rospy.Subscriber("/ps2_status", Ps2_msgs, self.call_check9)
        rospy.Subscriber("/imu/data_bno055", Imu, self.call_check10)
        rospy.Subscriber("/raw_odom", Odometry, self.call_check11)
        rospy.Subscriber("/odom", Odometry, self.call_check12)
        
        # # topic camera
        rospy.Subscriber("/camera/color/image_raw", Image, self.call_check13)
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.call_check14)
        rospy.Subscriber("/tag_detections_image_d435", Image, self.call_check15)

        self.bring_status    = rospy.Publisher('/manage_bring_status', ManageLaunch, queue_size=100)
        self.stt = ManageLaunch()
        self.stt.string = "bringup: [1-10]"

        # variable check 
        self.check1 = False
        self.check2 = False
        self.check3 = False
        self.check4 = False
        self.check5 = False
        self.check6 = False
        self.check7 = False
        self.check8 = False
        self.check9 = False
        self.check10 = False
        self.check11 = False  
        self.check12 = False    
        self.check13 = False 
        self.check14 = False    
        self.check15 = False      

    def call_check1(self,data): 
        if self.check1 == False : self.check1 = True
    def call_check2(self,data): 
        if self.check2 == False : self.check2 = True
    def call_check3(self,data): 
        if self.check3 == False : self.check3 = True
    def call_check4(self,data): 
        if self.check4 == False : self.check4 = True
    def call_check5(self,data): 
        if self.check5 == False : self.check5 = True 
    def call_check6(self,data): 
        if self.check6 == False : self.check6 = True
    def call_check7(self,data): 
        if self.check7 == False : self.check7 = True
    def call_check8(self,data): 
        if self.check8 == False : self.check8 = True
    def call_check9(self,data): 
        if self.check9 == False : self.check9 = True
    def call_check10(self,data): 
        if self.check10 == False : self.check10 = True
    def call_check11(self,data): 
        if self.check11 == False : self.check11 = True
    def call_check12(self,data): 
        if self.check12 == False : self.check12 = True
    def call_check13(self,data): 
        if self.check13 == False : self.check13 = True
    def call_check14(self,data): 
        if self.check14 == False : self.check14 = True
    def call_check15(self,data): 
        if self.check15 == False : self.check15 = True

    def raa(self):

        step = 1  
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        while not rospy.is_shutdown():
          # 1.check laser lms+tim
            if step == 1 :
                launch1 = roslaunch.parent.ROSLaunchParent(uuid, [self.tf_config_trienlam])
                launch1.start()
                launch2 = roslaunch.parent.ROSLaunchParent(uuid, [self.lms100_tim551])
                launch2.start()
                rospy.logwarn("1.check laser lms+tim")
                
                # rospy.sleep(3)
                # 3 seconds later
                # launch.shutdown()
                step = 2

            if step == 2 :
                if self.check2 == True and self.check3 == True and self.check4 == True :
                    rospy.logwarn("1.laser lms+tim : OK")
                    self.stt.data = 1
                    self.bring_status.publish(self.stt)
                    step = 3
                else: 
                    step = 2

          # 2.check detected_leg_clusters
            if step == 3 :
                launch3 = roslaunch.parent.ROSLaunchParent(uuid, [self.detec_leg])
                launch3.start()
                rospy.logwarn("2.check detected_leg_clusters")
                step = 4

            if step == 4 :
                if self.check5 == True:
                    rospy.logwarn("2.detected_leg_clusters : OK")
                    self.stt.data = 2
                    self.bring_status.publish(self.stt)
                    step = 5
                else: 
                    step = 4
                    

          # 3.check hector_odom_v3
            if step == 5 :
                launch4 = roslaunch.parent.ROSLaunchParent(uuid, [self.hector_odom_v3])
                launch4.start()
                rospy.logwarn("3.check hector_odom_v3")
                step = 6

            if step == 6 :
                if self.check6 == True:
                    rospy.logwarn("3.hector_odom_v3 : OK")
                    self.stt.data = 3
                    self.bring_status.publish(self.stt)
                    step = 7
                else: 
                    step = 6
                    
            
          # 4.check conver_lidar
            if step == 7 :
                launch5 = roslaunch.parent.ROSLaunchParent(uuid, [self.conver_lidar])
                launch5.start()
                rospy.logwarn("4.check conver_lidar")
                step = 8

            if step == 8 :
                if self.check7 == True:
                    rospy.logwarn("4.conver_lidar : OK")
                    self.stt.data = 4
                    self.bring_status.publish(self.stt)
                    step = 9
                else: 
                    step = 8
                    

          # 5.check connect base_8_2
            if step == 9 :
                launch6 = roslaunch.parent.ROSLaunchParent(uuid, [self.base_8_2])
                launch6.start()
                rospy.logwarn("5.check connect base_8_2")
                step = 10

            if step == 10 :
                if self.check8 == True and self.check9 == True:
                    rospy.logwarn("5.connect base_8_2 : OK")
                    self.stt.data = 5
                    self.bring_status.publish(self.stt)
                    step = 11
                else: 
                    step = 10
                    

          # 6.check odom imu_bno055
            if step == 11 :
                launch7 = roslaunch.parent.ROSLaunchParent(uuid, [self.imu_bno055])
                launch7.start()
                rospy.logwarn("6.check odom imu_bno055")
                step = 12

            if step == 12 :
                if self.check10 == True:
                    rospy.logwarn("6.odom imu_bno055 : OK")
                    self.stt.data = 6
                    self.bring_status.publish(self.stt)
                    step = 13
                else: 
                    step = 12
                    
          
          # 7.check odom_encoder
            if step == 13 :
                launch8 = roslaunch.parent.ROSLaunchParent(uuid, [self.odom_encoder])
                launch8.start()
                rospy.logwarn("7.check odom_encoder")
                step = 14

            if step == 14 :
                if self.check11 == True:
                    rospy.logwarn("7.odom odom_encoder : OK")
                    self.stt.data = 7
                    self.bring_status.publish(self.stt)
                    step = 15
                else: 
                    step = 14

          # 8.check ekf
            if step == 15 :
                launch9 = roslaunch.parent.ROSLaunchParent(uuid, [self.ekf])
                launch9.start()
                rospy.logwarn("8.check ekf")
                step = 16

            if step == 16 :
                if self.check12 == True:
                    rospy.logwarn("8.odom ekf : OK")
                    self.stt.data = 8
                    self.bring_status.publish(self.stt)
                    step = 17
                else: 
                    step = 16

          # 9.check D435
            if step == 17 :
                launch10 = roslaunch.parent.ROSLaunchParent(uuid, [self.D435])
                launch10.start()
                rospy.logwarn("9.check D435")
                step = 18

            if step == 18 :
                if self.check13 == True and self.check14 == True:
                    rospy.logwarn("9.odom D435 : OK")
                    self.stt.data = 9
                    self.bring_status.publish(self.stt)
                    step = 19
                else: 
                    step = 18

          # 10.check apriltag_d435
            if step == 19 :
                launch11 = roslaunch.parent.ROSLaunchParent(uuid, [self.apriltag_d435])
                launch11.start()
                rospy.logwarn("10.check apriltag_d435")
                step = 20

            if step == 20 :
                if self.check15 == True:
                    rospy.logwarn("10. odom apriltag_d435 : OK")
                    self.stt.data = 10
                    self.bring_status.publish(self.stt)
                    step = 21

                else: 
                    step = 20
          # DONE start bringup
            if step == 21 :
                self.stt.string = "bringup: [1-10] DONE !"
                self.stt.data = 10
                self.bring_status.publish(self.stt)
                rospy.sleep(0.5)

            self.rate.sleep()

def main():
    
    try:
        m = Start_launch()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()