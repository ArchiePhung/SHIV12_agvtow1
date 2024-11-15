import numpy as np

def SVD_algorithm(ls_pointMap, ls_pointRef):
    # Tập hợp A và B (ví dụ)
    A = np.array(ls_pointMap) # map
    B = np.array(ls_pointRef) # gương

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
    theta_g = -np.arctan2(R[1, 0], R[0, 0])

    return robot_position_global[0], robot_position_global[1], theta_g

    print(f"Ma trận quay R:\n{R}")
    print(f"Vector tịnh tiến t: {t}")
    print(f"Tọa độ trong hệ toàn cục: {robot_position_global}")
    print(f"Góc quay trong hệ toàn cục: {theta_g} độ")