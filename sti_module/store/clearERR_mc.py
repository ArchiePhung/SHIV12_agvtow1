#!/usr/bin/env python
# author : Anh Tuan - 5-10-2020

import rospy
from std_msgs.msg import Bool
from sti_msgs.msg import Status_base

class Start_launch:
    def __init__(self):
        rospy.init_node('reset_mc_10time', anonymous=True)
        self.rate = rospy.Rate(10)
   
        self.reset    = rospy.Publisher('/reset_mc', Bool, queue_size=100)
        ros::Subscriber subscribe5 = n.subscribe("/status_base", 10, callSub)
    
    def callSub(self,dat):
        dat

    def raa(self): 
        step = 1 
        # while not rospy.is_shutdown():
            # if step == 1 :
        for i in range(10):
            self.reset.publish(True)
            rospy.sleep(0.1)

                # step = 2     

            # self.rate.sleep()

        

def main():
    
    try:
        m = Start_launch()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()