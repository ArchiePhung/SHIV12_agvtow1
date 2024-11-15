#!/usr/bin/env python3
# update : BEE - 04-01-2023

import serial
import os
from message_pkg.msg import Status_port
import roslaunch
import rospy
import string
import subprocess, platform
import time

class CheckPhysical:
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('check_physical', anonymous=True)
        self.rate = rospy.Rate(1)

        self.port_lms100 = rospy.get_param('port_lms100', '')   
        self.port_tim551 = rospy.get_param('port_tim551', '')   
        self.port_camera = rospy.get_param('port_camera', '')  
        self.port_board  = rospy.get_param('port_board', '')    
        self.port_imu    = rospy.get_param('port_imu', '')
        self.port_motor = rospy.get_param('port_motor', '')
        self.port_lidar = rospy.get_param('port_lidar', '')

        self.pub_statusPort = rospy.Publisher('/status_port', Status_port, queue_size= 50)
        self.statusPort = Status_port()

        self.pre_time = time.time()

    def ethernet_check(self, address):
        try:
            # print address
            output = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=True)
            # print(output)
            result = str(output).find('time=') 
            # print ("time",result ) # yes : >0 
            if result != -1:
                return 1
            return 0
        except Exception as e:
            return 0

    def usbSerial_check_c3(self, nameport):
        try:
            output = subprocess.check_output("ls -{} {} {} {} {}".format('l','/dev/','|','grep', nameport ), shell=True)
            vitri = output.find("stibase")

            name = output[(vitri):(vitri+len(nameport))]
            # print(name)
            if name == nameport:
                return 1
            return 0
        except Exception as e:
            return 0

    def usbSerial_check(self, nameport):
        try:
            output = subprocess.check_output("ls {} {} {} {}".format('/dev/','|','grep', nameport ), shell=True)
            locate = str(output).find(nameport)
            if locate != -1:
                return 1
            return 0
        except Exception as e:
            return 0

    def usbCamera_check(self,nameport):
        try:
            output = subprocess.check_output("rs-enumerate-devices -{}".format('s'), shell=True)
            # print(output)
            vitri = str(output).find(nameport[1:])
            # name = output[(vitri):(vitri+len(nameport))]
            # print(result)
            # print(nameport[1:])
            if vitri > 1 : return 1
        except Exception as e:
            # print("no port")
            return 0

    def run(self):

        while not rospy.is_shutdown():
          # -- LMS100
            self.statusPort.lms100 = True
            # self.pre_time = time.time()
            # if self.ethernet_check(self.port_lms100) == 1:
            #     self.statusPort.lms100 = True
            # else:
            #     self.statusPort.lms100 = False
            # t = time.time() - self.pre_time

          # -- TiM551
            self.statusPort.tim551 = True
            # if self.ethernet_check(self.port_tim551) == 1:
            #     self.statusPort.tim551 = True
            # else:
            #     self.statusPort.tim551 = False

            # -- LIDAR
            if self.ethernet_check(self.port_lidar) == 1:
                self.statusPort.lidar = True
            else:
                self.statusPort.lidar = False

        #   # -- D435
            self.statusPort.camera = True
            # if self.usbCamera_check(self.port_camera) == 1:
            #     self.statusPort.camera = True
            # else:
            #     self.statusPort.camera = True
            
          # -- BOARD        
            if self.usbSerial_check(self.port_board) == 1:
                self.statusPort.board = True
            else:
                self.statusPort.board = False

          # -- IMU   
            self.statusPort.imu = True  
            # if self.usbSerial_check(self.port_imu) == 1:
            #      self.statusPort.imu = True
            # else:
            #     self.statusPort.imu = True

            self.statusPort.magLine = True 

            if self.usbSerial_check(self.port_motor) == 1:
                self.statusPort.motorLeft = True 
                self.statusPort.motorRight = True 

            else:
                self.statusPort.motorLeft = False 
                self.statusPort.motorRight = False 

            self.pub_statusPort.publish(self.statusPort)

            self.rate.sleep()

def main():
    print('Program starting')
    try:
        program = CheckPhysical()
        program.run()
    except rospy.ROSInterruptException:
        pass
    print('Programer stopped')

if __name__ == '__main__':
    main()