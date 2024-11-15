#!/usr/bin/env python
# author : Anh Tuan - 18-9-2020
# update : Anh Tuan - 28-9-2020

import serial
import os
from sti_msgs.msg import  ManageLaunch
import roslaunch
import rospy
import string
import subprocess, platform

class Start_launch:
    def __init__(self):
        rospy.init_node('reconnect', anonymous=True)
        self.rate = rospy.Rate(10)

        self.lms100_port = rospy.get_param('~lms100_port','')      
        self.mana_info    = rospy.Publisher('/manage_status', ManageLaunch, queue_size=100)
        self.stt = ManageLaunch()

        # variable check 
        self.lms100_check = False

   
    def call_check0(self,data): 
        self.stt = data


    def raa(self): 
        while not rospy.is_shutdown():

            self.rate.sleep()

def main():
    
    try:
        m = Start_launch()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()