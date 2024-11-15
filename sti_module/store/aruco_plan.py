#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist, Point
from apriltag_ros.msg import AprilTagDetectionArray

carrot = Point()
def getCarrot(msg):
    global carrot
    a = len(msg.detections)
    print "msg.detections= ", a
    for a in range(a):
        print "id= ", msg.detections[a-1].id[0]

    # if len(msg.detections) == 1:
    #     carrot = msg.detections[0].pose.pose.pose.position 
    # else :
    #     carrot = Point()

if __name__ == '__main__':
    try:
        rospy.init_node('aruco_plan')
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, getCarrot)
        pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 100)
        r = rospy.Rate(1)

        speed = Twist()

        while not rospy.is_shutdown():
            # print carrot
            # if carrot.z > 0.35 or carrot.z < -0.35:
            #     speed.linear.x  = carrot.z / 5
            #     speed.angular.z = -carrot.x / 2 
            #     print "a"
            # else :
            #     speed.linear.x  = 0
            #     speed.angular.z = 0 
            #     print "bbb"

            # print speed

            # pub_vel.publish(speed)
            r.sleep()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Initial pose test finished.")

#b1 : 