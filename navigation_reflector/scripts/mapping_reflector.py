#!/usr/bin/env python3

import signal
import sys
import os
from datetime import datetime

from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Int8
 
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

from collections import deque

from triangleAlgorithm import triangleAlgorithm
from svd_algorithm import SVD_algorithm

'''
- Đọc dữ liệu map đã lưu
- Nhận dữ liệu gương phát hiện hiện tại 
'''

class reflectorMap():
    def __init__(self, _id = 0, _x = 0., _y = 0.):
        self.id = _id
        self.x = _x
        self.y = _y

class dataReflector():
    def __init__(self, _x = 0., _y = 0., _dis = 0., _angle = 0.):
        self.map = reflectorMap()
        self.x_relative = _x
        self.y_relative = _y
        self.distance_reflector = _dis
        self.angle_reflector = _angle

class ReflectorMatching():
    def __init__(self, ls_reflector_):
        self.ls_reflector = ls_reflector_

        self.ls_reflectorMap = []

class DetectReflector():
    def __init__(self):
        rospy.init_node('mapping_reflector', anonymous = True)
        self.rate = rospy.Rate(20)

        # -- load data map
        self.dir_mapReflector = '/home/stivietnam/catkin_ws/src/navigation_reflector/data/map_reflector.json'
        self.lsReflector_inMap = []
        arr_kdtree_refInMap = []

        try:
            with open(self.dir_mapReflector, 'r') as file:
                data = json.load(file)
                for ref in data["info"]:
                    self.lsReflector_inMap.append(reflectorMap(ref["id"], ref["x"], ref["y"]))
                    arr_kdtree_refInMap.append([ref["id"], ref["x"], ref["y"]])

                arr_kdtree_refInMap = np.array(arr_kdtree_refInMap)
                self.kdtree = KDTree(arr_kdtree_refInMap[:, 1:3])

        except Exception as e:
            print("Read json file fail: ", e)

        self.have_map_old = False
        if len(self.lsReflector_inMap) > 2:
            self.have_map_old = True
        
        # -- param
        # self.x_init = rospy.get_param('~x_init', 0.)
        # self.y_init = rospy.get_param('~y_init', 0.)
        # self.r_init = rospy.get_param('~r_init', 0.)

        # -- param
        self.x_init = rospy.get_param('~x_init', 5.476)
        self.y_init = rospy.get_param('~y_init', 28.463)
        self.r_init = rospy.get_param('~r_init', radians(-178.85))

        self.use_sensorPose = rospy.get_param('~use_sensorPose', False)

        # -------- Topic Sub -------- #
        # --
        rospy.Subscriber('/info_raw_reflector', Raw_reflector, self.callback_infoRawReflector, queue_size = 10)
        self.date_raw_reflector = Raw_reflector()
        self.is_rawReflector = False

        if self.use_sensorPose:
            rospy.Subscriber('/sensorPose_reference', PoseStamped, self.callback_poseSensorReference, queue_size = 10)
            self.is_poseSensorRef = False
            self.poseSensorRef = Pose()

        # -- 
        self.process = 0
        self.status_match = False

        # -- 
        self.num_refStart = -1
        self.num_getPoseReflector = 0
        self.save_numRef = -1
        self.data_store = []


        # -- 
        self.savePose_robot = [0. ,0. ,0.]
        self.num_savePoseRobot = 0

    def callback_infoRawReflector(self, data):
        self.date_raw_reflector = data
        self.is_rawReflector = True

    def callback_poseSensorReference(self, data):
        self.poseSensorRef = data.pose
        self.is_poseSensorRef = True
    
    def distance_squared(self, point):
        return point.x**2 + point.y**2
    
    def calculate_reflector_position_in_map(self, x_ss, y_ss, r_ss, x_ref, y_ref):
        # Tính toán ma trận quay thủ công (cos(R), sin(R))
        cos_R = cos(r_ss)
        sin_R = sin(r_ss)

        x_m = x_ss + (cos_R*x_ref - sin_R*y_ref)
        y_m = y_ss + (sin_R*x_ref + cos_R*y_ref)

        return x_m, y_m
    
    def add_reflectorMap(self, ls_ref):
        # -- lấy id gương lớn nhất
        id_refAdd = 0
        for refMap in self.lsReflector_inMap:
            if refMap.id > id_refAdd:
                id_refAdd = refMap.id
        # -- 
        # -- thêm gương vào list cũ
        for ref in ls_ref:
            id_refAdd += 1
            self.lsReflector_inMap.append(reflectorMap(id_refAdd, ref[0], ref[1]))

        # -- ghi file
        now = datetime.now().strftime("%Y-%m-%d %H:%M")
        json_out = {"name": 'map_reflector', "time_update": str(now), "info": []}
        for info_ref in self.lsReflector_inMap:
            json_infoRef = {"id": info_ref.id, "x": info_ref.x, "y": info_ref.y}
            json_out["info"].append(json_infoRef)

        temp_file_path = self.dir_mapReflector + '.tmp' 
        try:
            with open(temp_file_path, 'w', encoding='utf-8') as temp_file:
                json.dump(json_out, temp_file, ensure_ascii=False, indent=4)
                temp_file.flush()
                os.fsync(temp_file.fileno())
            os.rename(temp_file_path, self.dir_mapReflector)

        except Exception as e:
            print("Write json file backup error", e)

    def run(self):
        while not rospy.is_shutdown():
            if self.process == 0:
                if self.is_rawReflector:
                    self.process = 1

            elif self.process == 1:
                if self.is_rawReflector == False:
                    self.rate.sleep()
                    continue
                                
                if self.use_sensorPose and self.is_poseSensorRef == False:
                    # print("No data sensor pose")
                    self.rate.sleep()
                    continue
                
                self.is_rawReflector = False

                # -- kiểm tra số lượng gương quét được
                num_reflector = len(self.date_raw_reflector.points)
                if num_reflector < 3:
                    print("The number of mirrors is less than 3")
                    self.rate.sleep()
                    continue

                # -- 

                # -- Lấy toạ độ gương lọc
                ls_refFilter = []
                for ref in self.date_raw_reflector.points:
                    ls_refFilter.append([ref.x, ref.y])

                # ls_refFilter = [[1.2, 2.3], [1.0, 4.5], [1.3, 2.5]]
                
                # - sắp xếp các gương theo gần đến xa
                arrange_reflector = sorted(self.date_raw_reflector.points, key=self.distance_squared)

                # -- Trường hợp không có gương trong map 
                if self.have_map_old == False:

                    if self.num_refStart == -1:
                        self.num_refStart = len(ls_refFilter)

                        for r in range(self.num_refStart):
                            self.data_store.append([0., 0.])

                    # -- lấy toạ độ trung bình cho các điểm gương
                    if self.num_refStart != len(ls_refFilter):
                        print("số lượng gương các lần đọc không phù hơp")
                        return

                    numave = 20
                    if self.num_getPoseReflector < numave:
                        print('.')
                        for index, ref in enumerate(ls_refFilter):
                            self.data_store[index][0] += ref[0]
                            self.data_store[index][1] += ref[1]

                        self.num_getPoseReflector += 1
                        self.rate.sleep()
                        continue

                    # -- Tính trung bình 20 lần đọc tạo độ gương
                    ls_refFilter_average = []
                    for ave_ref in self.data_store:
                        ls_refFilter_average.append([ave_ref[0]/numave, ave_ref[1]/numave])

                    print(ls_refFilter_average)

                    numref_before = len(self.lsReflector_inMap)
                    # Tính lại các vị trí gương
                    convert_posReflector = []
                    for ref in ls_refFilter_average:
                        x_cv, y_cv = self.calculate_reflector_position_in_map(self.x_init, self.y_init, self.r_init, ref[0], ref[1])
                        convert_posReflector.append([x_cv, y_cv])

                    # Thêm gương vào map
                    self.add_reflectorMap(convert_posReflector)
                    numref_after = len(self.lsReflector_inMap)
                    
                    print("add reflector done, number reflector new is ", numref_after - numref_before)
                    return
                
                # -- Trường hợp đã có gương trong map
                else:
                    # -- tim cac vị trí gương quét được trong map
                    # Tính lại các vị trí gương
                    convert_posReflector = []
                    for ref in ls_refFilter:
                        x_cv, y_cv = self.calculate_reflector_position_in_map(self.x_init, self.y_init, self.r_init, ref[0], ref[1])
                        convert_posReflector.append([x_cv, y_cv])

                    # print(convert_posReflector)
                    # Tìm các chỉ số của các điểm nằm trong bán kính 3 cm
                    ls_refMatchMap = []
                    numRefMatchMap = 0

                    for index, pointCV in enumerate(convert_posReflector):
                        indices = self.kdtree.query_ball_point(pointCV, 0.1)
                        if len(indices) > 0:
                            numRefMatchMap += 1
                        # print(indices)
                        ls_refMatchMap.append(indices)

                    # -- kiểm tra các điểm gương phát hiện
                    if numRefMatchMap < 3:
                        # print("numRefMatchMap < 3")
                        print("The number of mirrors matching is less than 3")
                        self.savePose_robot = [0. ,0. ,0.]
                        self.num_savePoseRobot = 0
                        self.rate.sleep()
                        continue

                    # -- 
                    if num_reflector > numRefMatchMap:
                        print("Number new reflector: ", num_reflector - numRefMatchMap)
                        # -- tính lại toạ độ
                        ls_refMap = []
                        ls_refSensor = []

                        ls_refNotMatchmap = []

                        for index, refMath in enumerate(ls_refMatchMap):
                            # print(refMath)
                            if len(refMath) > 0:
                                ind = refMath[0]
                                ls_refSensor.append(ls_refFilter[index])
                                ls_refMap.append([self.lsReflector_inMap[ind].x, self.lsReflector_inMap[ind].y])

                            else:
                                ls_refNotMatchmap.append(ls_refFilter[index])

                        xr, yr, phiR = SVD_algorithm(ls_refMap, ls_refSensor) #(m, m. degree)
                        # print(xr, yr, phiR)
                        self.x_reference, self.y_reference, self.r_reference = xr, yr, radians(phiR)

                        if self.num_savePoseRobot < 10:
                            self.num_savePoseRobot += 1

                            self.savePose_robot[0] += xr
                            self.savePose_robot[1] += yr
                            self.savePose_robot[2] += phiR
                            # -- 
                            self.rate.sleep()
                            continue
                        
                        x_ave = self.savePose_robot[0]/self.num_savePoseRobot
                        y_ave = self.savePose_robot[1]/self.num_savePoseRobot
                        phiR_ave = self.savePose_robot[2]/self.num_savePoseRobot

                        x_ave = 5.476
                        y_ave = 28.463
                        phiR_ave = -178.85
                        
                        print("pose sensor now: ", x_ave, y_ave, phiR_ave)
                        
                        next = None
                        while next != 'no' and next != 'yes':
                            try:
                                next = input("Add new reflector (yes/no): ")
                            except:
                                return

                        if next == 'no':
                            return
                        
                        numref_before = len(self.lsReflector_inMap)
                        # convert các toạ độ điểm gương unknow
                        convert_posReflectorNotMatch = []
                        for ref in ls_refNotMatchmap:
                            x_cv, y_cv = self.calculate_reflector_position_in_map(x_ave, y_ave, radians(phiR_ave), ref[0], ref[1])
                            convert_posReflectorNotMatch.append([x_cv, y_cv])

                        # print()
                        # Thêm gương vào map
                        self.add_reflectorMap(convert_posReflectorNotMatch)
                        numref_after = len(self.lsReflector_inMap)
                        print("add reflector done, number reflector new is ", numref_after - numref_before)
                        return

                    else:
                        print("Have no new reflector")

                print("---end loop---")

            self.rate.sleep()


def main():
    print ("--- Run Detect Reflector ---")
    program = DetectReflector()
    program.run()

    print ("--- close! ---")

if __name__ == '__main__':
    main()

