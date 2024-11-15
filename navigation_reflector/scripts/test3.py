from collections import deque

# Sử dụng deque để lưu trữ 10 lần lấy mẫu gần nhất
history = deque(maxlen=10)

def add_coordinates(coords):
    """
    Thêm một danh sách các tọa độ (x, y) vào history
    """
    history.append(coords)

def calculate_average():
    """
    Tính trung bình các tọa độ cho từng cặp (x, y) qua 10 lần lấy mẫu
    """
    if not history:
        return None

    num_samples = len(history)
    num_coords = len(history[0])  # Số lượng cặp tọa độ trong mỗi mẫu
    average_coords = []

    # Khởi tạo danh sách lưu giá trị trung bình (từng cặp tọa độ)
    sum_x = [0] * num_coords
    sum_y = [0] * num_coords
    
    # Cộng dồn giá trị của từng cặp tọa độ
    for sample in history:
        for i, (x, y) in enumerate(sample):
            sum_x[i] += x
            sum_y[i] += y

    # Tính trung bình cho từng cặp tọa độ (x, y)
    for i in range(num_coords):
        avg_x = sum_x[i] / num_samples
        avg_y = sum_y[i] / num_samples
        average_coords.append((avg_x, avg_y))
    
    return average_coords

# Ví dụ sử dụng
# Lần 1: Nhận tọa độ
add_coordinates([(1, 2), (3, 4), (5, 6)])
# Lần 2: Nhận tọa độ
add_coordinates([(2, 3), (4, 5), (6, 7)])
# Lần 3: Nhận tọa độ
add_coordinates([(3, 4), (5, 6), (7, 8)])

# Tính trung bình sau mỗi lần nhận
average_coords = calculate_average()
print("Trung bình tọa độ:", average_coords)