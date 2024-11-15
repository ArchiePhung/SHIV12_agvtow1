#!/usr/bin/env python
#Author AnhTuan
#Start 22/5/2020
# update 7/10 : vung theo cac toa do

from leg_tracker.msg import LegArray
from visualization_msgs.msg import Marker , MarkerArray
from std_msgs.msg import Empty , ColorRGBA
from geometry_msgs.msg import  Pose , Point
from sti_msgs.msg import *
from math import atan2, sin, cos, sqrt, fabs 
import rospy
import time
import threading
import signal

class Point_new:
    x = 0.
    y = 0.

INF = 10000.
polygon_zone1 = Point_new()
polygon_zone2 = Point_new()

# extreme = Point_new()
# extreme = Point_new(5, 4)
# print("extreme",extreme.x,extreme.y)

class Complex:
    def __init__(self, x, y):
        self.x = x
        self.y = y

a = Complex(1.0, -1.5)
b = Complex(2.0, -2.5)
c = Complex(3.0, -3.5)
hihi = [a,b,c]
print(hihi)


# polygon_zone1.x = 10 
# polygon_zone1.y = 1 
# polygon_zone2.x = 4 
# polygon_zone2.y = 5 
# print("hihi",polygon_zone1.x,polygon_zone1.y)
# print("haha",polygon_zone2.x,polygon_zone2.y)

class Leg_detection(threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.shutdown_flag = threading.Event()
        self.rate = rospy.Rate(15)
        rospy.Subscriber("/detected_leg_clusters",LegArray, self.call_sub1)
        rospy.Subscriber("/zone_3d",Safety_zone, self.call_sub2)
        self.marker_pub = rospy.Publisher('marker_guong', Marker, queue_size=1)

        self.zone_robot_pub = rospy.Publisher('zone_robot_new',Zone_lidar_2head, queue_size=10)
        self.zone_lidar_2head_pub = rospy.Publisher('zone_lidar_2head_new',Zone_lidar_2head, queue_size=10)
        self.legData = LegArray()
        self.is_detec_leg = False

      #get param 
        self.dx_robot = rospy.get_param('~dx_robot',0.32)
        self.dy_robot = rospy.get_param('~dy_robot',0.26)
        self.dx1 = rospy.get_param('~dx1',1)
        self.dx2 = rospy.get_param('~dx2',2)
        self.dx3 = rospy.get_param('~dx3',3)

      #zone ahead : zone 1
        self.Ax = rospy.get_param('~Ax',0)
        self.Ay = rospy.get_param('~Ay',0)
        self.Bx = rospy.get_param('~Bx',0)
        self.By = rospy.get_param('~By',0)
        self.Cx = rospy.get_param('~Cx',0)
        self.Cy = rospy.get_param('~Cy',0)
        self.Dx = rospy.get_param('~Dx',0)
        self.Dy = rospy.get_param('~Dy',0)

      #zone behind : zone 1
        self._Ax = rospy.get_param('~_Ax',0)
        self._Ay = rospy.get_param('~_Ay',0)
        self._Bx = rospy.get_param('~_Bx',0)
        self._By = rospy.get_param('~_By',0)
        self._Cx = rospy.get_param('~_Cx',0)
        self._Cy = rospy.get_param('~_Cy',0)
        self._Dx = rospy.get_param('~_Dx',0)
        self._Dy = rospy.get_param('~_Dy',0)

        self.zone_lidar = Safety_zone()
        self.zone_3d = Safety_zone()
        self.zone_robot = Zone_lidar_2head()
        self.zone_2head = Zone_lidar_2head()


    def call_sub1(self,data):
        self.legData = data
        if self.is_detec_leg == False : self.is_detec_leg = True
        # print "diem: " ,len(self.legData.legs)
        # print self.legData.legs[1]

    def call_sub2(self,data):
        self.zone_3d = data

    # Given three colinear points p, q, r, the function checks if 
    # point q lies on line segment 'pr' 
    def onSegment(self, p, q, r):
        if (q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y) ):
            return True
        return false

    # To find orientation of ordered triplet (p, q, r). 
    # The function returns following values 
    # 0 --> p, q and r are colinear 
    # 1 --> Clockwise 
    # 2 --> Counterclockwise 
    def orientation(self, p, q, r):
        val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
        
        if (val == 0): return 0   # colinear 
        elif val > 0 : return 1 # clock or counterclock wise
        else :         return 2 

    # The function that returns True if line segment 'p1q1' 
    # and 'p2q2' intersect. 
    def doIntersect(self, p1, q1, p2, q2) :
        # Find the four orientations needed for general and 
        # special cases 
        o1 = self.orientation(p1, q1, p2)
        o2 = self.orientation(p1, q1, q2)
        o3 = self.orientation(p2, q2, p1)
        o4 = self.orientation(p2, q2, q1)
    
        # General case 
        if (o1 != o2 and o3 != o4):
            return True
    
        # Special Cases 
        # p1, q1 and p2 are colinear and p2 lies on segment p1q1 
        if (o1 == 0 and self.onSegment(p1, p2, q1)): return True
    
        # p1, q1 and p2 are colinear and q2 lies on segment p1q1 
        if (o2 == 0 and self.onSegment(p1, q2, q1)): return True 
    
        # p2, q2 and p1 are colinear and p1 lies on segment p2q2 
        if (o3 == 0 and self.onSegment(p2, p1, q2)): return True 
    
        # p2, q2 and q1 are colinear and q1 lies on segment p2q2 
        if (o4 == 0 and self.onSegment(p2, q1, q2)): return True 
    
        return False # Doesn't fall in any of the above cases 
    

    


    # Returns true if the point p lies inside the polygon[] with n vertices 
    # def isInside( polygon[], n, p) { 
    #     # There must be at least 3 vertices in polygon[] 
    #     if (n < 3)  return false
    
    #     # Create a point for line segment from p to infinite 
    #     extreme = Point_new
    #     extreme = {INF, p.y} 
    
    #     # Count intersections of the above line with sides of polygon 
    #     int count = 0, i = 0; 
    #     do
    #     { 
    #         int next = (i+1)%n; 
    
    #         # Check if the line segment from 'p' to 'extreme' intersects 
    #         # with the line segment from 'polygon[i]' to 'polygon[next]' 
    #         if (doIntersect(polygon[i], polygon[next], p, extreme)) 
    #         { 
    #             # If the point 'p' is colinear with line segment 'i-next', 
    #             # then check if it lies on segment. If it lies, return true, 
    #             # otherwise false 
    #             if (orientation(polygon[i], p, polygon[next]) == 0) 
    #             return onSegment(polygon[i], p, polygon[next]); 
    
    #             count++; 
    #         } 
    #         i = next; 
    #     } while (i != 0); 
    
    #     # Return true if count is odd, false otherwise 
    #     return count&1;  # Same as (count%2 == 1) 
    # }  

    def run(self):
        while not self.shutdown_flag.is_set():
            print("hihi")


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
    rospy.init_node('safety_zone_v3')
    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
 
    print('Starting main program')
 
    # Start the job threads
    try:
        thread1 = Leg_detection(1)
        thread1.start()
        # thread2 = Connect_ros(2)
        # thread2.start()
 
        # Keep the main thread running, otherwise signals are ignored.
        while True:
            time.sleep(0.01)
 
    except ServiceExit:
        thread1.shutdown_flag.set()
        thread1.join()
        # thread2.shutdown_flag.set()
        # thread3.join()
    print('Exiting main program')
 
if __name__ == '__main__':
    main()