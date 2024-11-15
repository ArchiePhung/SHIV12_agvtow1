# from math import radians, degrees, tan, atan2

# def cot(a):
#     return 1 / tan(a)

# # Tọa độ các gương tương ứng trong hệ tọa độ bản đồ
# ls_point = [[-0.0955176369255314, -0.36555958785105896], [0.018701137028734358, -1.463147242194885], [0.7057881705319228, -1.9912794729850718]]

# angle = [4.455290874896955, 4.725025943917234, 5.053507154209072]
# angle_1 = [4.452362113436009, 4.7255489287053285, 5.052298119230044]

# print(degrees(angle_1[0] - angle[0]), degrees(angle_1[1] - angle[1]), degrees(angle_1[2] - angle[2]))


# # -- Xác định hệ tọa độ mới của gƣơng
# x1_ = ls_point[0][0] - ls_point[1][0]
# y1_ = ls_point[0][1] - ls_point[1][1]

# x3_ = ls_point[2][0] - ls_point[1][0]
# y3_ = ls_point[2][1] - ls_point[1][1]

# # -- Tính giá trị các cot(.)
# T12 = cot(angle[1] - angle[0])
# T23 = cot(angle[2] - angle[1])

# T31 = (1-T12*T23)/(T12+T23)

# # -- Xác định tọa độ tâm gƣơng thay thế
# x12_ = x1_ + T12*y1_
# y12_ = y1_ - T12*x1_

# x23_ = x3_ - T23*y3_
# y23_ = y3_ + T23*x3_

# x31_ = (x3_ + x1_) + T31*(y3_ - y1_)
# y31_ = (y3_ + y1_) - T31*(x3_ - x1_)

# # -- Xác định hệ số đặc trƣng của đƣờng tròn
# k31_ = x1_*x3_ + y1_*y3_ + T31*(x1_*y3_ - x3_*y1_)

# # -- Tính định thức
# D = (x12_ - x23_)*(y23_ - y31_) - (y12_ - y23_)*(x23_ - x31_)

# # -- Xác định vị trí và hƣớng của robot
# xr = ls_point[1][0] + (k31_*(y12_ - y23_)/D)
# yr = ls_point[1][1] + (k31_*(x23_ - x12_)/D)

# phiR = atan2(ls_point[1][1] - yr, ls_point[1][0] - xr) - angle[1]

# print(xr, yr, phiR)


import numpy as np

# Tập hợp A và B (ví dụ)
A = np.array([[15, 25], [20, 15], [10, 20]]) # map
B = np.array([[-5, 0], [5, 5], [0, -5]]) # gương

# Tính trung bình của A và B
mu_A = np.mean(A, axis=0)
mu_B = np.mean(B, axis=0)

# Tái căn giữa các điểm (A_n và B_n)
A_n = A - mu_A
B_n = B - mu_B

# Tính ma trận hiệp phương sai H
H = np.zeros((2, 2))

for i in range(len(A)):
    H += np.outer(A_n[i], B_n[i])

# Phân tích SVD trên ma trận H
U, S, Vt = np.linalg.svd(H)

# Tính ma trận quay R
R = Vt.T @ U.T

# Nếu cần điều chỉnh (det(R) = -1), sửa đổi Vt hoặc R để giữ R là ma trận quay hợp lệ
if np.linalg.det(R) < 0:
    Vt[1,:] *= -1
    R = Vt.T @ U.T

# Tính vector tịnh tiến t
t = -R @ mu_A + mu_B

# Tọa độ của robot trong hệ tọa độ robot (0, 0)
x_l, y_l = np.array([0, 0])

# Tính tọa độ của robot trong hệ tọa độ toàn cục (với chuyển vị)
robot_position_global = (np.linalg.inv(R) @ (-t)).T

# Tính góc quay theta_g
theta_g = -np.arctan2(R[1, 0], R[0, 0]) * (180 / np.pi)

print(f"Ma trận quay R:\n{R}")
print(f"Vector tịnh tiến t: {t}")
print(f"Tọa độ trong hệ toàn cục: {robot_position_global}")
print(f"Góc quay trong hệ toàn cục: {theta_g} độ")

