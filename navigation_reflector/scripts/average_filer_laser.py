#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanFilter:
    def __init__(self):
        # Khởi tạo node
        rospy.init_node('average_filter_node', anonymous=True)
        
        # Đăng ký subscriber cho topic dữ liệu scan
        self.scan_sub = rospy.Subscriber('/sick_safetyscanners/scan', LaserScan, self.scan_callback)
        
        # Đăng ký publisher cho topic dữ liệu trung bình
        self.avg_pub = rospy.Publisher('/nanoScan3_average_filtered', LaserScan, queue_size=10)

        # Danh sách để lưu trữ dữ liệu quét
        self.scan_data_list = []
        self.max_scans = 2  # Số lần quét cần nhận trước khi công bố 1 | 6

    def scan_callback(self, data):
        # Chuyển đổi dữ liệu scan thành numpy array
        current_ranges = np.array(data.ranges)

        # Lọc các giá trị vô hạn (infinity)
        current_ranges = np.where(current_ranges == float('Inf'), np.nan, current_ranges)

        # Thêm dữ liệu quét vào danh sách
        self.scan_data_list.append(current_ranges)

        # Nếu đã nhận đủ 5 dữ liệu, tính trung bình và công bố
        if len(self.scan_data_list) >= self.max_scans:
            # Tính trung bình cho tất cả dữ liệu quét
            avg_ranges = np.nanmean(np.array(self.scan_data_list), axis=0)

            # Tạo message LaserScan để công bố
            avg_scan = LaserScan()
            avg_scan.header.stamp = rospy.Time.now()
            avg_scan.header.frame_id = data.header.frame_id
            avg_scan.angle_min = data.angle_min
            avg_scan.angle_max = data.angle_max
            avg_scan.angle_increment = data.angle_increment
            avg_scan.time_increment = data.time_increment
            avg_scan.range_min = data.range_min
            avg_scan.range_max = data.range_max
            
            # Xuất bản toàn bộ dữ liệu quét
            avg_scan.ranges = avg_ranges.tolist()  # Gửi toàn bộ điểm trung bình
            avg_scan.intensities = data.intensities  # Giữ nguyên giá trị intensities

            # Công bố dữ liệu trung bình ra topic
            self.avg_pub.publish(avg_scan)

            # Xóa danh sách sau khi công bố
            self.scan_data_list.clear()

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        scan_filter = ScanFilter()
        scan_filter.spin()
    except rospy.ROSInterruptException:
        pass
