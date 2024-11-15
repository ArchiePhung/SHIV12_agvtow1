#!/usr/bin/env python3

from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Int8
 
from math import atan2, sin, cos, sqrt, fabs, degrees, isnan, radians
from math import pi as PI
import rospy
import time
import copy

import json

import numpy as np
from scipy.optimize import minimize

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from navigation_reflector.msg import Raw_reflector

from sick_safetyscanners.msg import ExtendedLaserScanMsg

'''
- Nhận tín hiệu bắt đầu
- Phát hiện, phân cụm các vùng gương phản xạ theo bán kính gương và cường độ gương
- tính toán các vị trí gương: toạ độ tương đối, tuyệt đối, khoảng cách + góc tới các gương
- gửi data lên ROS
'''

# class reflectorMap():
#     def __init__(self, _id = 0, _x = 0., _y = 0.):
#         self.id = _id
#         self.x = _x
#         self.y = _y

# class dataReflector():
#     def __init__(self, _x = 0., _y = 0., _dis = 0., _angle = 0.):
#         self.map = reflectorMap()
#         self.x_relative = _x
#         self.y_relative = _y
#         self.distance_reflector = _dis
#         self.angle_reflector = _angle

# class ReflectorMatching():
#     def __init__(self, ls_reflector_):
#         self.ls_reflector = ls_reflector_

#         self.ls_reflectorMap = []

class DetectReflector():
    def __init__(self):
        rospy.init_node('detect_reflector', anonymous = True)
        self.rate = rospy.Rate(50)
        
        # -- param
        self.radius_reflector = rospy.get_param('~radius_reflector', 0.03)
        self.tolerance_radiusRelector = 0.01

        self.diameter_reflector = 2.0*self.radius_reflector
        self.circumference_reflector = 2.0*self.radius_reflector*PI
        self.half_circumference_reflector = self.radius_reflector*PI

        self.min_scanningRadius = rospy.get_param('~min_scanningRadius', 0.2)
        self.max_scanningRadius = rospy.get_param('~max_scanningRadius', 30.)

        self.min_reflectionIntensity = rospy.get_param('~min_reflectionIntensity', 200)
        self.max_reflectionIntensity = rospy.get_param('~max_reflectionIntensity', 5000)

        # -- 
        self.x_base_to_lidar = rospy.get_param('~x_base_to_lidar', 0.483)
        self.y_base_to_lidar = rospy.get_param('~y_base_to_lidar', 0.0)
        self.r_base_to_lidar = rospy.get_param('~r_base_to_lidar', 0.0)

        # -------- Topic Sub -------- #
        # --
        rospy.Subscriber("/status_naviReflector", Int8, self.callback_statusNaviReflector)
        self.status_naviReflector = 0

        # -- 
        rospy.Subscriber("/sick_safetyscanners/extended_laser_scan", ExtendedLaserScanMsg, self.callback_scan)
        self.dataScan = ExtendedLaserScanMsg()
        self.is_scan = False

        # -------- Topic Pub -------- #
        self.pub_infoReflector = rospy.Publisher('/info_raw_reflector', Raw_reflector, queue_size= 10) 

        self.pub_dataFilter = rospy.Publisher('/scan_filter', LaserScan, queue_size= 10)

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        # -- 
        self.process = 0


    def callback_statusNaviReflector(self, data):
        self.status_naviReflector = data.data

    def callback_scan(self, data):
        self.dataScan = data

        # print(len(self.dataScan.intensities))
        self.is_scan = True

    def convertDataLidarToDescartes(self, angle_min, angle_increment, index, dis):
        theta = angle_min + index*angle_increment
        x = dis*cos(theta)
        y = dis*sin(theta)
        return x, y
    
    def constraint_distance_from_origin(self, O, points):
        x_O, y_O = O
        # Tính độ dài của tâm O so với (0, 0)
        center_distance = np.sqrt(x_O**2 + y_O**2)
        # Tính độ dài lớn nhất của các điểm so với (0, 0)
        max_point_distance = np.max(np.sqrt(points[:, 0]**2 + points[:, 1]**2))
        # Yêu cầu tâm O phải xa gốc hơn các điểm
        return center_distance - max_point_distance
    
    def loss_function(self, O, points, R):
        x_O, y_O = O  # Tọa độ của tâm O
        total_error = 0
        for point in points:
            x_i, y_i = point
            # Tính khoảng cách từ điểm đến tâm O
            distance = np.sqrt((x_i - x_O)**2 + (y_i - y_O)**2)
            # Sai số là chênh lệch giữa khoảng cách này và bán kính R
            total_error += (distance - R)**2
        return total_error
    
    def pub_marker(self, list_point):
        # Tạo Marker kiểu SPHERE_LIST để hiển thị danh sách các điểm tâm
        marker = Marker()

        # Đặt các thuộc tính cơ bản cho Marker
        marker.header.frame_id = 'nanoScan'  # Frame tham chiếu, ví dụ: "world" hoặc "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "circle_centers"
        marker.id = 0  # ID của marker
        marker.type = Marker.SPHERE_LIST  # Sử dụng SPHERE_LIST để hiển thị nhiều điểm
        marker.action = Marker.ADD  # Thao tác: thêm vào hiển thị

        # Kích thước của các hình cầu (tất cả các điểm tâm sẽ có cùng kích thước)
        marker.scale.x = 0.06  # Bán kính SPHERE trên trục x
        marker.scale.y = 0.06  # Bán kính SPHERE trên trục y
        marker.scale.z = 0.06  # Bán kính SPHERE trên trục z

        # Màu sắc của các hình cầu (RGBA)
        marker.color.r = 0.0  # Màu đỏ
        marker.color.g = 1.0  # Màu xanh lá
        marker.color.b = 0.0  # Màu xanh dương
        marker.color.a = 1.0  # Độ đậm của màu (1.0 là không trong suốt)

        # Thêm danh sách các điểm vào Marker
        for center in list_point:
            p = Point()
            p.x = center[0]
            p.y = center[1]
            p.z = 0.
            marker.points.append(p)

        self.marker_pub.publish(marker)


    def run(self):
        while not rospy.is_shutdown():
            if self.process == 0:
                if self.is_scan:
                    self.process = 1

            elif self.process == 1:
                if self.is_scan == False:
                    self.rate.sleep()
                    continue
                
                # reset 
                self.is_scan = False

                data_scan = copy.copy(self.dataScan)

                # -- 
                dis_arr = list(data_scan.laser_scan.ranges)

                # -- lọc các cụm data là True
                ls_reflektor = []
                for index, hri in enumerate(data_scan.reflektor_median):
                    condition_1 = data_scan.laser_scan.ranges[index] >= self.min_scanningRadius and data_scan.laser_scan.ranges[index] >= self.min_scanningRadius
                    condition_2 = hri
                    
                    if condition_1 and condition_2:
                        ls_reflektor.append((index, data_scan.laser_scan.ranges[index], data_scan.laser_scan.intensities[index]))


                # print(ls_reflektor) # (index, dis)
                # print('---+---')
                
                # -- Tính toán vị trí gương so với robot hiện tại
                print('====')
                ls_poseReflector = []

                msg_raw_reflector = Raw_reflector()
                msg_raw_reflector.header.frame_id = 'raw_reflector'
                msg_raw_reflector.header.stamp = rospy.Time.now()

                for reflector in ls_reflektor:
                    pose_reflector = Point()
                    # -- list các toạ độ
                    x, y = self.convertDataLidarToDescartes(data_scan.laser_scan.angle_min, data_scan.laser_scan.angle_increment, reflector[0], reflector[1])

                    print(x,y)

                    # -- chuyển sang tâm robot
                    x_cv = self.x_base_to_lidar + x*cos(self.r_base_to_lidar) - y*sin(self.r_base_to_lidar)
                    y_cv = self.y_base_to_lidar + x*sin(self.r_base_to_lidar) + y*cos(self.r_base_to_lidar)

                    # --
                    pose_reflector.x = x_cv
                    pose_reflector.y = y_cv
                    
                    ls_poseReflector.append([x, y])
                    msg_raw_reflector.points.append(pose_reflector)

                self.pub_marker(ls_poseReflector)
                self.pub_infoReflector.publish(msg_raw_reflector)

                # -- gửi dữ liệu các điểm gương quét đươc


            self.rate.sleep()



def main():
    print ("--- Run Detect Reflector ---")
    program = DetectReflector()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()




