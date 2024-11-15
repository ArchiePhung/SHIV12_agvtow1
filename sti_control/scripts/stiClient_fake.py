#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    - Fake lộ trình cho LAGV demo
    - Lặp lại 2 quy trình: 
       + Lấy kệ 2 >> trả hàng ở kệ 1 >> về sạc
       + Lấy kệ 1 >> trả kệ ở 2 >> về sạc 
"""

import roslib
roslib.load_manifest('sti_control')
import rospy

from sti_msgs.msg import *
from geometry_msgs.msg import *
import math
import time 

class Fake_stiClient():
    def __init__(self):
        # -- init node -- 
        rospy.init_node('stiClient_fake', anonymous=False)
        
        # -- node publish -- 
        self.pub_NNcmdRequest = rospy.Publisher('NN_cmdRequest', NN_cmdRequest, queue_size = 10)
        self.data_NNcmdRequest = NN_cmdRequest()

        self.rate = rospy.Rate(50.0)
        
        # -- node subcribe -- 
        rospy.Subscriber('NN_infoRespond' , NN_infoRespond, self.callBack_NNinfoRespond)
        self.data_NNinfoRespond = NN_infoRespond()
        
        # -- NN_cmdRequest message
        self.process = 0.0       # pass 
        self.target_id = 0       # pass
        self.target_x = 0.0      
        self.target_y = 0.0
        self.target_z = 0.0
        self.tag = 0             # pass 
        self.offset = 0.0
        self.list_id = []
        self.list_x = []
        self.list_y = []
        self.list_speed = []
        self.before_mission = 0
        self.after_mission = 0
        self.id_command = 0       # always = 0
        self.command = ""         # pass
 
        # -- biến hệ thống --
        self.DieTime_pub = time.time()           # thời gian chờ pub
        self.timeWait = time.time()              # thời gian chờ giữa 2 quy trình 
        self.process = 1                            # 1: quy trinh 1, 2: quy trinh 2, 3: thời gian chờ đổi quy trình 
        self.subprocess = 1                         # 1: sac -> ke, 2: ke -> ke, 3: ke -> sac
    
    # -- hàm nhận dữ liệu từ AGV-TF 
    def callBack_NNinfoRespond(self, data):
        self.data_NNinfoRespond = data

    def run(self):
        while not rospy.is_shutdown():
            if self.data_NNinfoRespond.mode == 2:                        # AGV ở chế độ tự động
                if self.process == 1:           # quy trinh 1
                    if self.subprocess == 1:    # -- từ sạc đến vị trí lấy kệ 2
                        if time.time() - self.DieTime_pub < 3:                   
                            self.data_NNcmdRequest.process = 0.0
                            self.data_NNcmdRequest.target_id = 0
                            self.data_NNcmdRequest.target_x = 6.084
                            self.data_NNcmdRequest.target_y = -3.158
                            self.data_NNcmdRequest.target_z = 1.5624
                            self.data_NNcmdRequest.tag = 0
                            self.data_NNcmdRequest.offset = 1.31
                            self.data_NNcmdRequest.list_id = [1, 3, 4, 0, 0]
                            self.data_NNcmdRequest.list_x = [9.514, 6.0 , 6.084, 0, 0]
                            self.data_NNcmdRequest.list_y = [-1.76, -1.825, -3.158, 0, 0]
                            self.data_NNcmdRequest.list_speed = [100, 100, 100, 0, 0]
                            self.data_NNcmdRequest.before_mission = 2
                            self.data_NNcmdRequest.after_mission = 1
                            self.data_NNcmdRequest.id_command = 0
                            self.data_NNcmdRequest.command = ''

                        else:
                            self.pub_NNcmdRequest.publish(self.data_NNcmdRequest)
                            if self.data_NNinfoRespond.task_status == 1:
                                self.subprocess = 2
                                self.DieTime_pub = time.time()

                    elif self.subprocess == 2:  # -- từ vị trí lấy kệ 2 về vị trí lấy kệ 1
                        if time.time() - self.DieTime_pub < 3:                   
                            self.data_NNcmdRequest.process = 0.0
                            self.data_NNcmdRequest.target_id = 0
                            self.data_NNcmdRequest.target_x = 9.562
                            self.data_NNcmdRequest.target_y = -3.658
                            self.data_NNcmdRequest.target_z = 1.5766
                            self.data_NNcmdRequest.tag = 0
                            self.data_NNcmdRequest.offset = 1.311
                            self.data_NNcmdRequest.list_id = [4, 3, 1, 2, 0]
                            self.data_NNcmdRequest.list_x = [6.084, 6.0 , 9.514, 9.562, 0]
                            self.data_NNcmdRequest.list_y = [-3.158, -1.825, -1.76, -3.658, 0]
                            self.data_NNcmdRequest.list_speed = [100, 100, 100, 100, 0]
                            self.data_NNcmdRequest.before_mission = 1
                            self.data_NNcmdRequest.after_mission = 2
                            self.data_NNcmdRequest.id_command = 0
                            self.data_NNcmdRequest.command = ''

                        else:
                            self.pub_NNcmdRequest.publish(self.data_NNcmdRequest)
                            if self.data_NNinfoRespond.task_status == 2:
                                self.subprocess = 3
                                self.DieTime_pub = time.time()
                    
                    else:                       # -- hạ kệ ở vị trí 1 rồi về sạc 
                        if time.time() - self.DieTime_pub < 3:                   
                            self.data_NNcmdRequest.process = 0.0
                            self.data_NNcmdRequest.target_id = 0
                            self.data_NNcmdRequest.target_x = 9.514
                            self.data_NNcmdRequest.target_y = -1.76
                            self.data_NNcmdRequest.target_z = 3.1405
                            self.data_NNcmdRequest.tag = 0
                            self.data_NNcmdRequest.offset = 1.909
                            self.data_NNcmdRequest.list_id = [2, 1, 0, 0, 0]
                            self.data_NNcmdRequest.list_x = [9.562, 9.514 , 0, 0, 0]
                            self.data_NNcmdRequest.list_y = [-3.658, -1.76, 0, 0, 0]
                            self.data_NNcmdRequest.list_speed = [100, 100, 0, 0, 0]
                            self.data_NNcmdRequest.before_mission = 2
                            self.data_NNcmdRequest.after_mission = 6
                            self.data_NNcmdRequest.id_command = 0
                            self.data_NNcmdRequest.command = ''

                        else:
                            self.pub_NNcmdRequest.publish(self.data_NNcmdRequest)
                            if self.data_NNinfoRespond.task_status == 6:
                                self.process = 3
                                self.DieTime_pub = time.time()
                                self.timeWait = time.time()
                                self.data_NNcmdRequest = NN_cmdRequest()

                elif self.process == 3:
                    if time.time() -self.timeWait < 30:   # ko cho AGV chay 
                        self.data_NNcmdRequest.process = 0.0
                        self.data_NNcmdRequest.target_id = 0
                        self.data_NNcmdRequest.target_x = 9.514
                        self.data_NNcmdRequest.target_y = -1.76
                        self.data_NNcmdRequest.target_z = 3.1405
                        self.data_NNcmdRequest.tag = 0
                        self.data_NNcmdRequest.offset = 1.909
                        self.data_NNcmdRequest.list_id = [1, 0, 0, 0, 0]
                        self.data_NNcmdRequest.list_x = [9.514, 0, 0, 0, 0]
                        self.data_NNcmdRequest.list_y = [-1.76, 0, 0, 0, 0]
                        self.data_NNcmdRequest.list_speed = [0, 0, 0, 0, 0]
                        self.data_NNcmdRequest.before_mission = 6
                        self.data_NNcmdRequest.after_mission = 6
                        self.data_NNcmdRequest.id_command = 0
                        self.data_NNcmdRequest.command = ''
                        self.pub_NNcmdRequest.publish(self.data_NNcmdRequest)
                    else:    # wait for 60s
                        self.process = 2
                        self.subprocess = 1
                        self.DieTime_pub = time.time()                   

                elif self.process == 2:                        # quy trinh 2
                    if self.subprocess == 1:                   # tu sac den ke 1 lay hang
                        if time.time() - self.DieTime_pub < 3:                   
                            self.data_NNcmdRequest.process = 0.0
                            self.data_NNcmdRequest.target_id = 0
                            self.data_NNcmdRequest.target_x = 9.562
                            self.data_NNcmdRequest.target_y = -3.658
                            self.data_NNcmdRequest.target_z = 1.5766
                            self.data_NNcmdRequest.tag = 0
                            self.data_NNcmdRequest.offset = 1.311
                            self.data_NNcmdRequest.list_id = [1, 2, 0, 0, 0]
                            self.data_NNcmdRequest.list_x = [9.514, 9.562 , 0, 0, 0]
                            self.data_NNcmdRequest.list_y = [-1.76, -3.658, 0, 0, 0]
                            self.data_NNcmdRequest.list_speed = [100, 100, 0, 0, 0]
                            self.data_NNcmdRequest.before_mission = 2
                            self.data_NNcmdRequest.after_mission = 1
                            self.data_NNcmdRequest.id_command = 0
                            self.data_NNcmdRequest.command = ''

                        else:
                            self.pub_NNcmdRequest.publish(self.data_NNcmdRequest)
                            if self.data_NNinfoRespond.task_status == 1:
                                self.subprocess = 2
                                self.DieTime_pub = time.time()

                    elif self.subprocess == 2:               # mang hang tu ke 1 sang ke 2
                        if time.time() - self.DieTime_pub < 3:                   
                            self.data_NNcmdRequest.process = 0.0
                            self.data_NNcmdRequest.target_id = 0
                            self.data_NNcmdRequest.target_x = 6.084
                            self.data_NNcmdRequest.target_y = -3.158
                            self.data_NNcmdRequest.target_z = 1.5624
                            self.data_NNcmdRequest.tag = 0
                            self.data_NNcmdRequest.offset = 1.31
                            self.data_NNcmdRequest.list_id = [2, 1, 3, 4, 0]
                            self.data_NNcmdRequest.list_x = [9.562, 9.514 , 6.0, 6.084, 0]
                            self.data_NNcmdRequest.list_y = [-3.658, -1.76, -1.825, -3.158, 0]
                            self.data_NNcmdRequest.list_speed = [100, 100, 100, 100, 0]
                            self.data_NNcmdRequest.before_mission = 1
                            self.data_NNcmdRequest.after_mission = 2
                            self.data_NNcmdRequest.id_command = 0
                            self.data_NNcmdRequest.command = ''

                        else:
                            self.pub_NNcmdRequest.publish(self.data_NNcmdRequest)
                            if self.data_NNinfoRespond.task_status == 2:
                                self.subprocess = 3
                                self.DieTime_pub = time.time()

                    else:                       # -- hạ kệ ở vị trí 2 rồi về sạc 
                        if time.time() - self.DieTime_pub < 3:                   
                            self.data_NNcmdRequest.process = 0.0
                            self.data_NNcmdRequest.target_id = 0
                            self.data_NNcmdRequest.target_x = 9.514
                            self.data_NNcmdRequest.target_y = -1.76
                            self.data_NNcmdRequest.target_z = 3.1405
                            self.data_NNcmdRequest.tag = 0
                            self.data_NNcmdRequest.offset = 1.909
                            self.data_NNcmdRequest.list_id = [4, 3, 1, 0, 0]
                            self.data_NNcmdRequest.list_x = [6.084, 6.0, 9.514, 0, 0]
                            self.data_NNcmdRequest.list_y = [-3.158, -1.825, -1.76, 0, 0]
                            self.data_NNcmdRequest.list_speed = [100, 100, 100, 0, 0]
                            self.data_NNcmdRequest.before_mission = 2
                            self.data_NNcmdRequest.after_mission = 6
                            self.data_NNcmdRequest.id_command = 0
                            self.data_NNcmdRequest.command = ''

                        else:
                            self.pub_NNcmdRequest.publish(self.data_NNcmdRequest)
                            if self.data_NNinfoRespond.task_status == 6:
                                self.process = 4
                                self.DieTime_pub = time.time()
                                self.timeWait = time.time()
                                self.data_NNcmdRequest = NN_cmdRequest()

                elif self.process == 4:
                    if time.time() - self.timeWait < 30:
                        self.data_NNcmdRequest.process = 0.0
                        self.data_NNcmdRequest.target_id = 0
                        self.data_NNcmdRequest.target_x = 9.514
                        self.data_NNcmdRequest.target_y = -1.76
                        self.data_NNcmdRequest.target_z = 3.1405
                        self.data_NNcmdRequest.tag = 0
                        self.data_NNcmdRequest.offset = 1.909
                        self.data_NNcmdRequest.list_id = [1, 0, 0, 0, 0]
                        self.data_NNcmdRequest.list_x = [9.514, 0, 0, 0, 0]
                        self.data_NNcmdRequest.list_y = [-1.76, 0, 0, 0, 0]
                        self.data_NNcmdRequest.list_speed = [0, 0, 0, 0, 0]
                        self.data_NNcmdRequest.before_mission = 6
                        self.data_NNcmdRequest.after_mission = 6
                        self.data_NNcmdRequest.id_command = 0
                        self.data_NNcmdRequest.command = ''                          
                        self.pub_NNcmdRequest.publish(self.data_NNcmdRequest)                        
                    else:    # wait for 60s
                        self.process = 1
                        self.subprocess = 1
                        self.DieTime_pub = time.time()

            elif self.data_NNinfoRespond.mode == 1:
                self.data_NNcmdRequest = NN_cmdRequest()
                self.DieTime_pub = time.time()

            self.rate.sleep()

def main():
	print('Program starting')
	program = Fake_stiClient()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()
