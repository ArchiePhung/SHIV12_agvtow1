#!/usr/bin/env python3

import os
from datetime import datetime
 
from math import atan2, sin, cos, sqrt, fabs, degrees, isnan
from math import pi as PI
import rospy
import time

import json

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, Pose, TransformStamped

import tf
import tf2_ros

class reflectorMap():
    def __init__(self, _id = 0, _x = 0., _y = 0.):
        self.id = _id
        self.x = _x
        self.y = _y


class ShowMapReflector():
    def __init__(self):
        rospy.init_node('show_map_reflector', anonymous = True)
        self.rate = rospy.Rate(10)

        # -- load data map
        self.dir_mapReflector = '/home/stivietnam/catkin_ws/src/navigation_reflector/data/map_reflector.json'

        # -- topic pub
        self.marker_pub = rospy.Publisher('/visualization_marker_mapReflector', Marker, queue_size=10)

        rospy.on_shutdown(self.deleteAllMarker)

        self.list_id = None

    def deleteAllMarker(self):
        for i in range(5):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "cylinder_shapes"  # Namespace chung của các marker
            marker.action = Marker.DELETEALL  # Xóa tất cả các markers trong namespace nà
            # Gửi Marker với hành động DELETE ALL
            self.marker_pub.publish(marker)
            rospy.sleep(0.05)

    def read_and_pubMarker(self):
        lsReflector_inMap = []
        ls_id = []
        try:
            with open(self.dir_mapReflector, 'r') as file:
                data = json.load(file)
                for ref in data["info"]:
                    lsReflector_inMap.append(reflectorMap(ref["id"], ref["x"], ref["y"]))

                    ls_id.append(ref["id"])

                if self.list_id != ls_id:
                    if self.list_id == None:
                        self.list_id = ls_id
                    else:
                        #-- kiem tra co phan tu trong mang
                        for id_n in self.list_id:
                            if id_n in ls_id:
                                pass
                            else:
                                print("id delete: ", id_n)
                                try:
                                    # -- xoa phan tu
                                    # Tạo một Marker để reset
                                    marker = Marker()
                                    marker.header.frame_id = "map"
                                    marker.header.stamp = rospy.Time.now()
                                    marker.ns = "cylinder_shapes"
                                    marker.id = id_n  # ID của Marker cần xóa
                                    marker.action = Marker.DELETE  # Hành động DELETE để xóa Marker

                                    # Gửi Marker với hành động DELETE
                                    self.marker_pub.publish(marker)

                                except:
                                    print("error delete")
                                    pass

                        self.list_id = ls_id
                        
                        
        except Exception as e:
            print("Read json file fail: ", e)

        for ref in lsReflector_inMap:
            # Tạo một marker mới cho từng hình trụ
            marker = Marker()
            marker.header.frame_id = "map"  # Khung tọa độ (frame)
            marker.header.stamp = rospy.Time.now()
            marker.ns = "cylinder_shapes"  # Namespace
            marker.id = ref.id  # ID của marker
            marker.type = Marker.CYLINDER  # Loại marker là hình trụ

            marker.action = Marker.ADD  # Thêm marker mới

            # Thiết lập tọa độ
            marker.pose.position = Point(ref.x, ref.y, 0.)  # Tọa độ (x, y, z)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0  # Định hướng

            marker.scale.x = 0.07  # Đường kính hình trụ
            marker.scale.y = 0.07  # Đường kính hình trụ
            marker.scale.z = 0.6  # Chiều cao hình trụ

            # Thiết lập màu sắc
            marker.color.a = 0.5  # Độ trong suốt (1.0 là không trong suốt)
            marker.color.r = 1.0  # Màu đỏ
            marker.color.g = 1.0  # Màu xanh lá
            marker.color.b = 1.0  # Màu xanh dương

            # Xuất bản marker
            self.marker_pub.publish(marker)    

            self.rate.sleep()    

    def run(self):
        while not rospy.is_shutdown():
            self.read_and_pubMarker()

def main():
    print ("--- Run Show Map Reflector ---")
    program = ShowMapReflector()
    program.run()
    print ("--- close! ---")

if __name__ == '__main__':
    main()
