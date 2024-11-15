#!/usr/bin/env python
# author : Anh Tuan - 16/9/2020

from sti_msgs.msg import *
from std_msgs.msg import *
import roslaunch

import rospy
import time
import threading
import signal

class Start_launch:
    def __init__(self):
        rospy.init_node('manage_launch', anonymous=True)
        self.rate = rospy.Rate(10)
        rospy.Subscriber("test_node", Float32, self.call_modbus)
        self.is_scan_received = False

    def call_modbus(self,data):
        if self.is_scan_received == False : self.is_scan_received = True

    def raa(self):
        step = 1  
        while not rospy.is_shutdown():
            # print "run"
            if step == 1 :
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/stivietnam/catkin_ws/src/sti_module/launch/test_node.launch"])
                launch.start()
                rospy.loginfo("started")

                # rospy.sleep(3)
                # 3 seconds later
                # launch.shutdown()
                step = 2

            if step == 2 :
                if self.is_scan_received == True :
                    rospy.loginfo("ok")
                    # step = 3

            self.rate.sleep()

def main():
    
    try:
        m = Start_launch()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()