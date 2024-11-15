#!/usr/bin/env python
# author : Anh Tuan - 14/5/2020

import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from sti_msgs.msg import Modbus_0x , Modbus_4x_read , Modbus_4x_write

import rospy
import time
import threading
import signal

class Connect_modbus(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()

        self.rate = rospy.Rate(100)


    def run(self):
        while not self.shutdown_flag.is_set():
            print "run"

            self.rate.sleep()
        print('Thread #%s stopped' % self.threadID)

class Connect_ros(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        self.rate = rospy.Rate(100)
        
        modbus_pub_0x = rospy.Publisher('modbus_0x',Modbus_0x , queue_size=50)
        modbus_pub_4x_read = rospy.Publisher('modbus_4x_read',Modbus_4x_read , queue_size=50)
        rospy.Subscriber("modbus_4x_write", Modbus_4x_write, self.call_modbus)

    def call_modbus(self,data):
        self.msg_scan =  data
        if self.is_scan_received == False : self.is_scan_received = True


    def run(self):
        while not self.shutdown_flag.is_set():
            print "hihi"

            self.rate.sleep()
        print('Thread #%s stopped' % self.threadID)

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
        thread1 = Connect_modbus(1)
        thread1.start()
        thread2 = Connect_ros(2)
        thread2.start()
 
        # Keep the main thread running, otherwise signals are ignored.
        while True:
            time.sleep(0.01)
 
    except ServiceExit:
        thread1.shutdown_flag.set()
        thread1.join()
        thread2.shutdown_flag.set()
        thread3.join()
    print('Exiting main program')
 
if __name__ == '__main__':
    main()