#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author : Phùng Quý Dương - Archie Phùng
# Date: 23-05-2024

"""
    > Get CPU Info
	> Get Wifi Info
	> Get time run of NUC after boot
	> Auto connect wifi if it get lost
	> Send number of losting wifi to Debug file
"""

import os
import psutil
import rospy
import time
from std_msgs.msg import Int16
from sti_msgs.msg import *

class NUC_Info:
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('nuc_info', anonymous= True) # , NoRosout = True
        self.rate = rospy.Rate(10)

        self.pub_NUC = rospy.Publisher('/nuc_info', Nuc_info, queue_size= 10)            # 1 topic tương ứng với 1 kiểu dữ liệu.
        self.NUC_info = Nuc_info()

        self.time_read = 0.5
        self.name_card = rospy.get_param("name_card", "wlo2")
        self.name_card = "wlo2"
        self.runOnce = 0
        self.start_time = time.time()
        self.step = 1
        self.time_checkWifi = time.time()
        self.uptime = 0

    def get_cpu_usage(self, time_read):
        try:
            cpu_usage = psutil.cpu_percent(time_read)
            return cpu_usage
        except Exception:
            return 0
        
    def get_cpu_temp(self):
        try:
            temperature = psutil.sensors_temperatures().get('coretemp')[0].current
            return temperature
        except Exception:
            return 0

    def get_ram_usage(self):
        """
        Obtains the absolute number of RAM bytes currently in use by the system.
        :returns: System RAM usage in bytes.
        :rtype: int
        """
        try:
            ram_usage = int(psutil.virtual_memory().total - psutil.virtual_memory().available)
            return ram_usage
        except Exception:
            return 0
        
    def get_ram_total(self):
        """
        Obtains the total amount of RAM in bytes available to the system.
        :returns: Total system RAM in bytes.
        :rtype: int
        """
        try:
            ram_total = int(psutil.virtual_memory().total)
            return ram_total
        except Exception:
            return 0
        

    def get_ram_usage_pct(self):
        """
        Obtains the system's current RAM usage.
        :returns: System RAM usage as a percentage.
        :rtype: float
        """
        try:
            ram_percent = psutil.virtual_memory().percent
            return ram_percent
        except Exception:
            return 0

    def get_qualityWifi(self, name_card): # int
        try:
            pos_quality = '0'
            pos_signal = '0'
            output = os.popen("iwconfig {}".format(name_card)).read()
            pos_quality = str(output).find('Link Quality=')
            pos_signal = str(output).find('Signal level=')
            # pos_bitrate = str(output).find('Bit Rate=')
            # pos_txpower = str(output).find('Tx-Power=')
            # -
            if pos_quality >= 0:
                quality_data = str(output)[pos_quality+13:pos_quality+15]
            # print ("quality_data: ", int(quality_data))
            
            # -
            if pos_signal >= 0:
                signal_data = str(output)[pos_signal+13:pos_signal+16]
            # print ("signal_data: ", int(signal_out)) 

            # -
            # if pos_bitrate >= 0:
            #     bitrate_data = str(output)[pos_bitrate+13:pos_bitrate+16]
            # # print ("bitrate_data: ", int(bitrate_data)) 

            # # -
            # if pos_txpower >= 0:
            #     txpwower_data = str(output)[pos_txpower+13:pos_txpower+16]
            # print ("txpwower_data: ", int(txpwower_data)) 

            return int(quality_data), int(signal_data)
        except Exception:
            return 0, 0

    def convert_intTotime(self, time):
        str_time = ""
        time_hour = int(time/3600)
        time = time - time_hour*3600
        time_minute = int(time/60)
        time_second = time - time_minute*60

        if time_hour == 0:
            if time_minute == 0:
                str_time = str(time_second) + "s"
            else:
                str_time = str(time_minute) + "m" + str(time_second) + "s"
        else:
            str_time = str(time_hour) + "h" + str(time_minute) + "m" + str(time_second) + "s"
            
        return str_time

    def run(self):
        while not rospy.is_shutdown():
            # -- CPU usage
            self.NUC_info.cpu_usage = self.get_cpu_usage(self.time_read)

            # -- CPU temp
            self.NUC_info.cpu_temp = self.get_cpu_temp()

            # -- RAM usage
            self.NUC_info.ram_usage = int(self.get_ram_usage() / 1024 / 1024)
            self.NUC_info.ram_usage = round(self.NUC_info.ram_usage/1000, 1)

            # -- RAM total
            self.NUC_info.ram_total = int(self.get_ram_total() / 1024 / 1024)
            self.NUC_info.ram_total = round(self.NUC_info.ram_total/1000, 1)
            
            # -- RAM percent
            self.NUC_info.ram_percent = self.get_ram_usage_pct()

            # -- Wifi
            self.NUC_info.wifi_quality, self.NUC_info.wifi_signal = self.get_qualityWifi(self.name_card)

            if self.step == 1:		
                if self.NUC_info.wifi_quality == 0 or self.NUC_info.wifi_signal == 0:
                    if self.runOnce == 1:
                        self.runOnce = 2
                        self.start_time = time.time()
                        os.system("nmcli radio wifi off")
                        self.time_checkWifi = time.time()
                        
                    elif self.runOnce == 2:
                        if time.time() - self.start_time > 1:
                            os.system("nmcli radio wifi on")
                            self.step = 2
                            self.runOnce = 0
                else:
                    self.runOnce = 1

            elif self.step == 2:              # check trạng thái đá kết nối được wifi chưa sau 5s
                if time.time() - self.time_checkWifi > 5:
                    if self.NUC_info.wifi_quality == 0 or self.NUC_info.wifi_signal == 0:
                        self.runOnce = 1
                        self.step = 1
                        
                if self.NUC_info.wifi_quality != 0 and self.NUC_info.wifi_signal != 0:
                    self.step = 1
                    self.runOnce = 1

            # -- Time run
            if time.time() - self.start_time >= 1:
                self.start_time = time.time()
                self.uptime = self.uptime + 1
            
            self.NUC_info.uptime = self.convert_intTotime(self.uptime)

            self.pub_NUC.publish(self.NUC_info)
            self.rate.sleep()

def main():
	print('Program starting')

	program = NUC_Info()
	program.run()

	print('Programer stopped')

if __name__ == '__main__':
    main()