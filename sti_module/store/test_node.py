#!/usr/bin/env python
# author : Anh Tuan - 16/9/2020

from sti_msgs.msg import *
from std_msgs.msg import Float32 
import roslaunch
import rospy
import time
import threading
import signal

class Check_node(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        self.rate = rospy.Rate(10)
        
        self.test = rospy.Publisher('test_node',Float32 , queue_size=50)

    def run(self):
        a = 0.
        while not self.shutdown_flag.is_set():
            print "hihi"
            a = a + 1
            # rospy.sleep(0.5)
            self.test.publish(a)

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
    rospy.init_node('test_node')
    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:
        thread2 = Check_node(1)
        thread2.start()
 
        # Keep the main thread running, otherwise signals are ignored.
        while True:
            time.sleep(0.01)
 
    except ServiceExit:
        thread2.shutdown_flag.set()
        thread2.join()
    print('Exiting main program')
 
if __name__ == '__main__':
    main()