std_msgs/Int16 status    # Trang thai base: (0 - unknowns) (1 - ok) (2 - error)
std_msgs/Int16 connect_error   # Lỗi: ko kết nối với thiết bị: (1 - IMU) (2 - PS2) (3 - IMU + PS2) (ko có số - connected -> Lost connect PC)
std_msgs/Int16 run_error       # Lỗi: đèn Driver báo: (1 - driver trái) (2 - Diver phải) | Gửi tốc độ nhưng ko có tín hiệu encoder thay đổi(3 - động cơ trái) (4 - động cơ phải)