#!/usr/bin/env python3

import os
from datetime import datetime

from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Int8

import tf
from tf.transformations import quaternion_from_euler
 
from math import atan2, sin, cos, sqrt, fabs, degrees, isnan, radians
from math import pi as PI
import rospy
import time
import copy

import json

import numpy as np
from scipy.spatial import KDTree
from scipy.optimize import minimize

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, Pose
from navigation_reflector.msg import Raw_reflector

from itertools import combinations

from collections import deque, Counter


from triangleAlgorithm import triangleAlgorithm
from svd_algorithm import SVD_algorithm

'''
- Tính vị trí AGV dựa vào thông tin các điểm gương phát hiện
- Gửi dữ liệu lên ROS
'''

class reflectorMap():
    def __init__(self, _id = 0, _x = 0., _y = 0.):
        self.id = _id
        self.x = _x
        self.y = _y

class NavigationReflector():
    def __init__(self):
        rospy.init_node('navigation_reflector', anonymous = True)
        self.rate = rospy.Rate(30)

        # -- load data map
        self.dir_mapReflector = '/home/stivietnam/catkin_ws/src/navigation_reflector/data/map_reflector.json'
        self.lsReflector_inMap = []
        arr_kdtree_refInMap = []
        arr_edge = []

        try:
            with open(self.dir_mapReflector, 'r') as file:
                data = json.load(file)
                for index, ref in enumerate(data["info"]):
                    self.lsReflector_inMap.append(reflectorMap(ref["id"], ref["x"], ref["y"]))
                    arr_kdtree_refInMap.append([ref["id"], ref["x"], ref["y"]])
                    arr_edge.append([ref["x"], ref["y"], index])

                arr_kdtree_refInMap = np.array(arr_kdtree_refInMap)
                self.kdtree = KDTree(arr_kdtree_refInMap[:, 1:3])

                self.ls_edge_map = list(combinations(arr_edge, 2))

        except Exception as e:
            print("Read json file fail: ", e)
            return
        
        # -- param
        self.x_reference = 0.
        self.y_reference = 0.
        self.r_reference = 0.

        # -------- Topic Sub -------- #
        # --
        rospy.Subscriber('/info_raw_reflector', Raw_reflector, self.callback_infoRawReflector, queue_size = 10)
        self.date_raw_reflector = Raw_reflector()
        self.is_rawReflector = False

        # -------- Topic Pub -------- #
        # --
        self.pub_rawPoseReflector = rospy.Publisher('/raw_poseReflector', Point, queue_size= 10) 

        # -- tf
        self.br = tf.TransformBroadcaster()
        self.save_timePubTf = time.time()
        self.rate_pubTf = 1.

        # -- 
        self.request_naviByReflector = 1

        self.process = 0

        # -- Lọc toạ độ
        self.num_samples = 5
        self.x_readings = np.zeros(self.num_samples)
        self.y_readings = np.zeros(self.num_samples)
        self.r_readings = np.zeros(self.num_samples)

        # Chỉ số hiện tại của lần đọc
        self.read_index = 0

        # Tổng của các giá trị x, y, r (để tính trung bình)
        self.x_total = 0
        self.y_total = 0
        self.r_total = 0

        self.data_average = []


    def callback_infoRawReflector(self, data):
        self.date_raw_reflector = data
        self.is_rawReflector = True

    def callback_poseSensorReference(self, data):
        self.poseSensorRef = data.pose
        self.is_poseSensorRef = True
    
    def distance_squared(self, point):
        return point.x**2 + point.y**2
    
    def distance_from_origin(self, point):
        x = point[0]
        y = point[1]
        # x, y = point
        return sqrt(x**2 + y**2)
    
    def calculate_reflector_position_in_map(self, x_ss, y_ss, r_ss, x_ref, y_ref):
        # Tính toán ma trận quay thủ công (cos(R), sin(R))
        cos_R = cos(r_ss)
        sin_R = sin(r_ss)

        x_m = x_ss + (cos_R*x_ref - sin_R*y_ref)
        y_m = y_ss + (sin_R*x_ref + cos_R*y_ref)

        return x_m, y_m
    
    def Smoothing_data(self, new_x, new_y, new_r):
        self.x_total -= self.x_readings[self.read_index]
        self.y_total -= self.y_readings[self.read_index]
        self.r_total -= self.r_readings[self.read_index]
        
        self.x_readings[self.read_index] = new_x
        self.y_readings[self.read_index] = new_y
        self.r_readings[self.read_index] = new_r
        
        # Cộng giá trị mới vào tổng
        self.x_total += new_x
        self.y_total += new_y
        self.r_total += new_r
        
        # Cập nhật chỉ số
        self.read_index = (self.read_index + 1) % self.num_samples  # Vòng lại chỉ số nếu đạt giới hạn
        
        # Tính trung bình
        x_avg = self.x_total / self.num_samples
        y_avg = self.y_total / self.num_samples
        r_avg = self.r_total / self.num_samples
        
        return x_avg, y_avg, r_avg
    
    def Average_data(self, new_x, new_y, new_r):
        # Thêm dữ liệu mới vào danh sách
        self.data_average.append([new_x, new_y, new_r])
        
        # Giữ lại chỉ 10 giá trị gần nhất
        if len(self.data_average) > self.num_samples:
            self.data_average.pop(0)  # Xóa giá trị cũ nhất nếu số lượng lớn hơn 10
        
        # Tính trung bình
        avg_x = np.mean([r[0] for r in self.data_average])
        avg_y = np.mean([r[1] for r in self.data_average])
        avg_r = np.mean([r[2] for r in self.data_average])
        
        return avg_x, avg_y, avg_r

    def send_tf(self, x, y, yaw):
        # if time.time() - self.save_timePubTf > 1/self.rate_pubTf:
        # Chuyển đổi từ yaw sang quaternion
        q = quaternion_from_euler(0.0, 0.0, yaw)  # roll và pitch là 0.0
        # Phát phép biến đổi động
        self.br.sendTransform((x, y, 0.0), q, rospy.Time.now(), "base_footprintRef", "map")

            # self.save_timePubTf = time.time()

    def run(self):
        while not rospy.is_shutdown():
            if self.process == 0:
                ck = 0
                # -- kiểm tra số lượng gương trong MAP
                if self.is_rawReflector:
                    print("data raw reflector: oke")
                    ck += 1
                # -- kiểm tra có dữ liệu gương chưa 
                if len(self.lsReflector_inMap) > 3:
                    print("data reflector in MAP: oke")
                    ck += 1

                if ck == 2:
                    self.process = 1

            elif self.process == 1:
                if self.is_rawReflector == False:
                    # print("No data reflector")
                    self.rate.sleep()
                    continue
                
                self.is_rawReflector = False

                # -- kiểm tra số lượng gương quét được
                num_reflector = len(self.date_raw_reflector.points)
                if num_reflector < 3:
                    print("The number of mirrors is less than 3")
                    self.rate.sleep()
                    continue
                
                # print("start --- ")

                # -- yêu cầu nội suy vị trí AGV từ các điểm gương
                if self.request_naviByReflector == 1:
                    # -- Lấy toạ độ gương lọc
                    ls_refFilter = []
                    for index, ref in enumerate(self.date_raw_reflector.points):
                        ls_refFilter.append([ref.x, ref.y, index])

                    # -- sap xep
                    # ls_refFilter = sorted(ls_refFilterRaw, key=self.distance_from_origin)
                    # print(ls_refFilter)

                    # -- tách cạnh
                    ls_edge_reflector = list(combinations(ls_refFilter, 2))

                    # -- kiểm tra các cạnh bằng nhau của 2 tập hợp
                    tolerance_dis = 0.05 # sai số khoảng cách
                    ls_edgeMap_dis = [] # chứa các cạnh map giống kc với các gương quét được

                    for edge_map in self.ls_edge_map:
                        dis = sqrt((edge_map[0][0] - edge_map[1][0])**2 + (edge_map[0][1] - edge_map[1][1])**2)
                        edge_map_dis = []
                        for index, edge_ref in enumerate(ls_edge_reflector):
                            dis_edge_ref = sqrt((edge_ref[0][0] - edge_ref[1][0])**2 + (edge_ref[0][1] - edge_ref[1][1])**2)
                            if fabs(dis - dis_edge_ref) < tolerance_dis:
                                ls_edgeMap_dis.append([edge_map, edge_ref[0][2], edge_ref[1][2]])

                    # -- tách các trường hợp ra 3 cạnh
                    ls_triangle_reflector = list(combinations(ls_edgeMap_dis, 3))
                    arr_temp = []

                    for triangle in ls_triangle_reflector:
                        ar = []
                        for edge_ref in list(combinations(triangle, 2)):
                            point_same = None
                            index_same = None
                            # -- tìm điểm chung và toạ độ chung
                            if point_same is None and edge_ref[0][0][0][2] == edge_ref[1][0][0][2]:
                                point_same = edge_ref[0][0][0]

                            if point_same is None and edge_ref[0][0][0][2] == edge_ref[1][0][1][2]:
                                point_same = edge_ref[0][0][0]

                            if point_same is None and edge_ref[0][0][1][2] == edge_ref[1][0][0][2]:
                                point_same = edge_ref[0][0][1]

                            if point_same is None and edge_ref[0][0][1][2] == edge_ref[1][0][1][2]:
                                point_same = edge_ref[0][0][1]

                            if point_same is None:
                                break
                            
                            if index_same is None and edge_ref[0][1] == edge_ref[1][1]:
                                index_same = edge_ref[0][1]

                            if index_same is None and edge_ref[0][1] == edge_ref[1][2]:
                                index_same = edge_ref[0][1]

                            if index_same is None and edge_ref[0][2] == edge_ref[1][1]:
                                index_same = edge_ref[0][2]

                            if index_same is None and edge_ref[0][2] == edge_ref[1][2]:
                                index_same = edge_ref[0][2]

                            if index_same is None:
                                break

                            ar.append([point_same, index_same])

                        if len(ar) == 3:
                            for i in ar:
                                arr_temp.append(i)

                    # print(arr_temp)
                    # -- tinh xac xuat xuat hien
                    probability_indexref_inMapRef = [[] for _ in range(len(ls_refFilter))]
                    for ref in arr_temp:
                        for i in range(len(ls_refFilter)):
                            if i == ref[1]:
                                probability_indexref_inMapRef[i].append(ref[0][2])
                                break 

                    ls_refMap = []
                    ls_refSensor = []

                    # # -- tinh xác xuất
                    for index, pro_ref in enumerate(probability_indexref_inMapRef):
                        if pro_ref:
                            # Đếm số lần xuất hiện của từng số
                            count = Counter(pro_ref)
                            # Tính tổng số phần tử trong mảng
                            total_elements = len(pro_ref)
                            # Tính xác suất
                            probabilities = {num: freq / total_elements for num, freq in count.items()}

                            index_with_highest_prob = max(probabilities, key=probabilities.get)

                            ls_refSensor.append([ls_refFilter[index][0], ls_refFilter[index][1]])
                            ls_refMap.append([self.lsReflector_inMap[index_with_highest_prob].x, self.lsReflector_inMap[index_with_highest_prob].y])


                    print("--------------------------")
                    print("ls_refSensor")
                    print(ls_refSensor)
                    print("ls_refMap")
                    print(ls_refMap)

                    if len(ls_refSensor) < 3:
                        print("The number of matching mirrors is less than 3, request_naviByReflector = 1")
                        self.rate.sleep()
                        continue

                    xr, yr, phiR = SVD_algorithm(ls_refMap, ls_refSensor)
                    # -- send tf
                    self.send_tf(xr, yr, phiR)

                    self.x_reference, self.y_reference, self.r_reference = xr, yr, phiR

                    x_sensor, y_sensor, r_sensor = self.Average_data(xr, yr, phiR)

                    print("num ref matching: ", len(ls_refSensor), "| Pose: ", x_sensor, y_sensor, degrees(r_sensor))

                    self.request_naviByReflector = 2


                # -- yêu cầu tính toạ độ AGV bằng gương
                elif self.request_naviByReflector == 2:
                    # -- Lấy toạ độ gương lọc
                    ls_refFilterRaw = []
                    for ref in self.date_raw_reflector.points:
                        ls_refFilterRaw.append([ref.x, ref.y])

                    # -- sap xep
                    ls_refFilter = sorted(ls_refFilterRaw, key=self.distance_from_origin)
                    # print("guong local")
                    # print(ls_refFilter)

                    # -- tim cac vị trí gương quét được trong map
                    # Tính lại các vị trí gương
                    convert_posReflector = []
                    for ref in ls_refFilter:
                        x_cv, y_cv = self.calculate_reflector_position_in_map(self.x_reference, self.y_reference, self.r_reference, ref[0], ref[1])
                        convert_posReflector.append([x_cv, y_cv])

                    # Tìm các chỉ số của các điểm nằm trong bán kính 3 cm
                    ls_refMatchMap = []
                    numRefMatchMap = 0

                    for pointCV in convert_posReflector:
                        indices = self.kdtree.query_ball_point(pointCV, 0.6)
                        if len(indices) > 0:
                            numRefMatchMap += 1
                        # print(indices)
                        ls_refMatchMap.append(indices)

                    # -- kiểm tra các điểm gương phát hiện
                    if numRefMatchMap < 3:
                        self.request_naviByReflector = 1
                        print("The number of matching mirrors is less than 3, request_naviByReflector = 2")
                        self.rate.sleep()
                        # -- thêm các điểm gương mới dựa vào vị trí khởi tạo
                        continue

                    # -- tính lại toạ độ
                    ls_refMap = []
                    ls_refSensor = []

                    for index, refMath in enumerate(ls_refMatchMap):
                        # print(refMath)
                        if len(refMath) > 0:
                            ind = refMath[0]
                            ls_refSensor.append(ls_refFilter[index])
                            ls_refMap.append([self.lsReflector_inMap[ind].x, self.lsReflector_inMap[ind].y])

                    # print(ls_refMap)
                    # print(ls_refSensor)
                    xr, yr, phiR = SVD_algorithm(ls_refMap, ls_refSensor)
                    # xr, yr, phiR = triangleAlgorithm(ls_refMap, ls_refSensor)
                    # -- send tf
                    self.send_tf(xr, yr, phiR)

                    self.x_reference, self.y_reference, self.r_reference = xr, yr, phiR

                    x_sensor, y_sensor, r_sensor = self.Average_data(xr, yr, phiR)
                    
                    # print("raw:    ", self.x_reference, self.y_reference, self.r_reference)
                    print("num ref matching: ", len(ls_refSensor), "| Pose: ", x_sensor, y_sensor, degrees(r_sensor))
                    # self.pub_rawPoseReflector.publish(Point(xr, yr, phiR))

            self.rate.sleep()

        print("---end loop---")

def main():
    print ("--- Run Detect Reflector ---")
    program = NavigationReflector()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()

