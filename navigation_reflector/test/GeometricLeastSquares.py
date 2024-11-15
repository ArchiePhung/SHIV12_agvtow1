import numpy as np
from scipy.optimize import minimize
from functools import partial

# Hàm tính tổng bình phương sai số giữa các điểm và đường tròn có bán kính cố định
def calc_fixed_radius_error(center, points, radius):
    distances = np.sqrt((points[:, 0] - center[0])**2 + (points[:, 1] - center[1])**2)
    return np.sum((distances - radius)**2)  # Tổng bình phương sai số

# Hàm ràng buộc rằng tâm đường tròn phải xa hơn so với các điểm dữ liệu từ cảm biến
def sensor_distance_constraint(center, points):
    distance_to_center = np.sqrt(center[0]**2 + center[1]**2)  # Khoảng cách từ máy quét đến tâm
    distances_to_points = np.sqrt(points[:, 0]**2 + points[:, 1]**2)  # Khoảng cách từ máy quét đến các điểm
    return distance_to_center - np.max(distances_to_points)  # Ràng buộc

# Hàm tìm tọa độ tâm với bán kính cố định và điều kiện ràng buộc
def fit_circle_with_fixed_radius_and_sensor_constraint(points, known_radius):
    initial_guess = [np.mean(points[:, 0]), np.mean(points[:, 1])]
    constraints = {'type': 'ineq', 'fun': partial(sensor_distance_constraint, points=points)}
    result = minimize(calc_fixed_radius_error, initial_guess, args=(points, known_radius), constraints=constraints)
    return result.x

# Dữ liệu laser mới
laser_data = np.array([
    [ 0.16171359, -1.53852458],
    [ 0.17513342, -1.53705479],
    [ 0.18853991, -1.53546796],
    [ 0.20193205, -1.53376419],
    [ 0.21530881, -1.53194363]
])

# Bán kính của đường tròn đã biết
known_radius = 0.03

# Tìm tọa độ tâm
xc, yc = fit_circle_with_fixed_radius_and_sensor_constraint(laser_data, known_radius)
print(f"Tâm của đường tròn: ({xc:.2f}, {yc:.2f}) với bán kính cố định: {known_radius}")
