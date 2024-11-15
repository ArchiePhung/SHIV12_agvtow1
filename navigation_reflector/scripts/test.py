# #!/usr/bin/env python

# import rospy
# from sensor_msgs.msg import LaserScan
# from laser_filters import LaserScanMedianFilter

# class LaserFilterNode:
#     def __init__(self):
#         # Khởi tạo node
#         rospy.init_node('laser_filter_node')

#         # Khởi tạo publisher và subscriber
#         self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
#         self.scan_pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=10)

#         # Khởi tạo bộ lọc trung vị
#         self.median_filter = LaserScanMedianFilter(window_size=5)

#     def scan_callback(self, msg):
#         # Lọc dữ liệu quét
#         filtered_scan = self.median_filter.update(msg)
#         # Xuất dữ liệu đã được lọc
#         self.scan_pub.publish(filtered_scan)

#     def run(self):
#         rospy.spin()

# if __name__ == '__main__':
#     try:
#         laser_filter_node = LaserFilterNode()
#         laser_filter_node.run()
#     except rospy.ROSInterruptException:
#         pass


import numpy as np
from scipy.spatial import KDTree
from itertools import combinations
from collections import Counter

from math import sqrt, fabs

# Hàm tính ICP cải tiến
def icp(A, B, max_iterations=2000, tolerance=1e-6):
    R = np.eye(2)  # Khởi tạo ma trận quay là ma trận đơn vị
    t = np.zeros(2)  # Khởi tạo vector tịnh tiến là (0,0)
    prev_error = float('inf')  # Sai số ban đầu là vô hạn
    
    for i in range(max_iterations):
        # Tìm các điểm gần nhất trong B tương ứng với các điểm trong A
        tree = KDTree(B)
        distances, indices = tree.query(A)
        closest_points = B[indices]
        
        # Tính trọng tâm của các điểm trong A và các điểm gần nhất trong B
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(closest_points, axis=0)
        
        # Dịch các điểm về tọa độ gốc (loại bỏ trọng tâm)
        AA = A - centroid_A
        BB = closest_points - centroid_B
        
        # Tính ma trận H để thực hiện SVD
        H = AA.T @ BB
        
        # Phân tích giá trị kỳ dị (SVD)
        U, S, Vt = np.linalg.svd(H)
        R_new = Vt.T @ U.T
        
        # Đảm bảo phép quay là đúng chiều (nếu có sai lệch)
        if np.linalg.det(R_new) < 0:
            Vt[1, :] *= -1
            R_new = Vt.T @ U.T
        
        # Tính vector tịnh tiến
        t_new = centroid_B.T - R_new @ centroid_A.T
        
        # Áp dụng phép biến đổi lên A
        A_transformed = (R_new @ A.T).T + t_new
        
        # Tính sai số khớp
        mean_error = np.mean(np.linalg.norm(A_transformed - closest_points, axis=1))
        
        # Kiểm tra nếu sai số đã đủ nhỏ
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
    
    return R_new, t_new, mean_error

def combine_with_indices(arr):
    indices = range(len(arr))
    return [(arr[i], arr[j], i, j) for i, j in combinations(indices, 2)]

# Hàm tính vị trí và góc robot
def calculate_robot_position_and_angle(R, t):
    # Ma trận quay ngược chiều
    R_inv = R.T
    # Tính vị trí của robot
    P_robot = -R_inv @ t
    # Tính góc quay của robot
    theta = np.arctan2(R[1, 0], R[0, 0])
    theta_degrees = np.degrees(theta)
    return P_robot, theta_degrees

# Tọa độ các gương trong bản đồ (map frame)
B = np.array([
    [-0.0033432891444972897, -3.216449435768278, 1], 
    [2.614396011129963, -1.7914939302770627, 2], 
    [2.224713165331259, 1.916355703245505, 3], 
    [0.2123530062141657, 5.429101502300616, 4], 
    [-2.59293786995556, 7.014585689657155, 5], 
    [-4.915596913440369, 5.704866721527245, 6], 
    [-2.928767857945922, 0.481755303844539, 7]
])

# Tọa độ các gương phát hiện trong robot frame
A = np.array([
    [3.885319772353813, -6.530627145892985, 0], 
    [0.8879318747519694, 3.191448646502917, 1], 
    [-1.9440326938988954, 2.3573564432359553, 2]
])


# -- 
ls_edge_map = list(combinations(B, 2))
ls_edge_reflector = list(combinations(A, 2))

print(len(ls_edge_map))
print(len(ls_edge_reflector))

ls_edgeMap_dis = []

# for i in range(len(ls_edge_reflector)):
#     ls_edgeMap_dis.append([])

# print(ls_edgeMap_dis)

for edge_map in ls_edge_map:
    dis = sqrt((edge_map[0][0] - edge_map[1][0])**2 + (edge_map[0][1] - edge_map[1][1])**2)

    print("dis edge map: ", dis)
    edge_map_dis = []
    for index, edge_ref in enumerate(ls_edge_reflector):
        dis_edge_ref = sqrt((edge_ref[0][0] - edge_ref[1][0])**2 + (edge_ref[0][1] - edge_ref[1][1])**2)
        print("dis edge reflector: ", dis_edge_ref)
        if fabs(dis - dis_edge_ref) < 0.1:
            ls_edgeMap_dis.append([edge_map, edge_ref[0][2], edge_ref[1][2]])

print(ls_edgeMap_dis)


ls_triangle_reflector = list(combinations(ls_edgeMap_dis, 3))

# print(ls_edge_reflector)
# print(ls_edgeMap_dis)

arr = []

print(len(ls_triangle_reflector))
# (array([10, 20,  1]), array([15, 25,  2])), 2, 3], [(array([10, 20,  1]), array([20, 15,  7])), 1, 2], [(array([10, 20,  1]), array([20, 15,  7])), 1, 3])
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
            arr.append(i)

probability_indexref_inMapRef = [[] for _ in range(len(A))]

for ref in arr:
    for i in range(len(A)):
        if i == ref[1]:
            probability_indexref_inMapRef[i].append(ref[0][2])
            break 
            
# -- tinh xác xuất
for index, pro_ref in enumerate(probability_indexref_inMapRef):

    print("Guong co index: ", index)
    # Đếm số lần xuất hiện của từng số
    count = Counter(pro_ref)
    # Tính tổng số phần tử trong mảng
    total_elements = len(pro_ref)
    # Tính xác suất
    probabilities = {num: freq / total_elements for num, freq in count.items()}

    for num, prob in probabilities.items():
        print(f"Số {num} có xác suất: {prob:.2%}")

# Tạo tất cả các tập con kích thước 3 từ tập B
# subset_size = A.shape[0]
# subsets = list(combinations(B, subset_size))

# Lưu kết quả khớp tốt nhất
# best_error = float('inf')
# best_R, best_t = None, None
# best_subset = None

# # Khớp từng tập con với tập A
# for subset in subsets:
#     subset = np.array(subset)
#     R, t, error = icp(A, subset)
    
#     # Kiểm tra nếu sai số khớp là nhỏ nhất
#     if error < best_error:
#         best_error = error
#         best_R, best_t = R, t
#         best_subset = subset

# # In ra kết quả tập con khớp tốt nhất
# print(f"Tập con khớp tốt nhất: \n{best_subset}")
# print(f"Ma trận quay tốt nhất R: \n{best_R}")
# print(f"Vector tịnh tiến tốt nhất t: \n{best_t}")
# print(f"Sai số khớp: {best_error}")

# # Chuyển đổi các gương phát hiện được sang hệ tọa độ map
# A_in_map_frame = (best_R @ A.T).T + best_t
# print(f"Tọa độ của gương trong hệ tọa độ map: \n{A_in_map_frame}")

# # Tính vị trí và góc của robot
# P_robot, robot_angle = calculate_robot_position_and_angle(best_R, best_t)
# print(f"Vị trí của robot trong hệ tọa độ bản đồ: {P_robot}")
# print(f"Góc quay của robot (theo độ): {robot_angle}")

