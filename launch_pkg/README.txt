Các phần sẽ thay thế:
- Phát hiện vật cản.
- Điều hướng: 
	+ move_base.
	+ sti_navigation
=> các gói, node loại bỏ:
	- Điều hướng:
		+ /move_base
		+ /goal_control_v7
		+ /sti_navigation

	- Phát hiện vật cản:
		+ 
Thứ tự khởi động vận hành bình thường:

0, Load toàn bộ biến số + Xóa log ROS.
1, Kiểm tra cổng kết nối.
2, Chế độ tự kết nối lại.
3, Màn hình HMI + sysnatic.
4, Mạch Main.
5, Mạch MC.
6, Mạch SC.
7, Mạch OC.
8, Mạch HC.
9, Cảm biến đường từ.
10, Camera.
11, Lidar LMS100.
12, Lidar TIM551.
13, tf.
14, Odom encoder.
15, hector odom.
16, convert lidar
17, ekf.
18, safety zone lidar.
19, map server.
21, amcl.
22, robot pose.
23, navigation.
24, parking control.
25, client UDP.
26, control.
27, Log file.

Thứ tự khởi động để quét bản đồ.
