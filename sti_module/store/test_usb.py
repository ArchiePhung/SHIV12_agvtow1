# python -m pip install pyserial
# pip install urllib3

import serial
import rospy
import os


# try:
#     ser = serial.Serial('/dev/stibase_mc82', 57600)
#     print(ser.name)
#     print(ser.is_open)
#     ser.close()
#     rospy.sleep(1)
#     print(ser.is_open)

#     ser = serial.Serial('/dev/stibase_main82', 57600)
#     print(ser.name)
#     print(ser.is_open)
#     ser.close()
#     rospy.sleep(1)
#     print(ser.is_open)

# except serial.serialutil.SerialException:
#     print("no port")
#     pass

# ret = os.system("ping -c 3 -W 3000 192.168.11.14") # 3 time
# print(ret)
# if ret != 0:
#     print "pc still alive"

# import os
# hostname = "192.168.11.14"
# response = os.system("ping -c 1 " + hostname)
# if response == 0:
#     pingstatus = "Network Active"
# else:
#     pingstatus = "Network Error"


import subprocess, platform
sHost = "192.168.11.19"
# def pingOk(sHost):
try:
    output = subprocess.check_output("ping -{} 1 -{} 3000 {}".format('c', 'W', sHost), shell=False)
    print(output)
except Exception, e:
    print("huhu")
    pass
        # return False

    # return True

# print(pingOk(192.168.11.14))



# import subprocess

# cmd = "rs-enumerate-devices"

# # returns output as byte string
# returned_output = subprocess.check_output(cmd)

# # using decode() function to convert byte string to string
# print('Current date is:', returned_output.decode("utf-8"))

# # find : Intel RealSense D435I
# result = returned_output.find('Intel RealSense D435Igg') 
# print ("d435",result ) # -1 , 51