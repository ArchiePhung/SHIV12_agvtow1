#!/usr/bin/env python
import subprocess
import time
import argparse

name_card = "wlp3s0"
address = "192.168.1.1"

while True:

    # try:
    #     db = subprocess.check_output("iwconfig {}".format(name_card), shell=True)
    #     # print(ping)
    #     vitri = db.find("Signal level")
    #     time_ping = db[(vitri+13):(vitri+16)]
    #     print("db= {}".format(float(time_ping)))

    # except Exception, e:
    #     print("no db")

    # try:
    #     ping = subprocess.check_output("ping -{} 1 {}".format('c',address), shell=True)
    #     # print(ping)
    #     vitri = ping.find("time")
    #     time_ping = ping[(vitri+5):(vitri+9)]
    #     print("time= {}".format(float(time_ping)))

    # except Exception, e:
    #     print("no ping")

    # try:
    #     sensors = subprocess.check_output("sensors", shell=True)
    #     vitri = sensors.find("Package id")
    #     temperature = sensors[(vitri+16):(vitri+20)]
    #     print("temperature= {}".format(float(temperature)))

    # except Exception, e:
    #     print("no temperature")

    nameport ="stibase_main82"
    # print(len(nameport))
    try: 
        output = subprocess.check_output("ls -{} {} {} {} {}".format('l','/dev/','|','grep', nameport), shell=True)
        vitri = output.find("stibase")
        # print(vitri)
        name = output[(vitri):(vitri+len(nameport))]
        print(name)
        if name == nameport :
            print("hihi")
        else:
            print("huhu")
        
    except Exception, e:
        print("port")

    time.sleep(1)