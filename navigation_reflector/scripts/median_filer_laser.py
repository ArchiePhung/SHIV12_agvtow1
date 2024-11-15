#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import time

class LaserFilterNode:
    def __init__(self):
        # Khởi tạo node
        rospy.init_node('median_filter_node')

        # Khởi tạo publisher và subscriber
        rospy.Subscriber('/scan_lms100', LaserScan, self.scan_callback)
        self.scan_pub = rospy.Publisher('/scan_lms100_median_filtered', LaserScan, queue_size=10)

        # Kích thước cửa sổ cho bộ lọc trung vị
        self.window_size_ranges = 5 # Giảm kích thước cửa sổ để tránh vượt quá biên dữ liệu
        self.window_size_intensities = 3

    def scan_callback(self, msg):
        t = time.time()
        # Lấy dữ liệu quét
        ranges = np.array(msg.ranges)
        intensities = np.array(msg.intensities)

        # Kiểm tra xem ranges có dữ liệu không
        if len(ranges) > 0:
            # Áp dụng bộ lọc trung vị cho khoảng cách với numpy
            filtered_ranges = np.copy(ranges)
            for i in range(len(ranges)):
                # Tạo cửa sổ lọc, chú ý tránh vượt quá biên
                window_start = max(0, i - self.window_size_ranges // 2)
                window_end = min(len(ranges), i + self.window_size_ranges // 2 + 1)
                filtered_ranges[i] = np.median(ranges[window_start:window_end])
        else:
            filtered_ranges = ranges

        # Áp dụng bộ lọc trung vị cho cường độ nếu có dữ liệu
        if len(intensities) > 0:
            filtered_intensities = np.copy(intensities)
            for i in range(len(intensities)):
                window_start = max(0, i - self.window_size_intensities // 2)
                window_end = min(len(intensities), i + self.window_size_intensities // 2 + 1)
                filtered_intensities[i] = np.median(intensities[window_start:window_end])
        else:
            filtered_intensities = intensities

        # Cập nhật dữ liệu quét đã được lọc
        msg.ranges = filtered_ranges
        msg.intensities = filtered_intensities

        # Xuất dữ liệu đã được lọc
        self.scan_pub.publish(msg)

        print(time.time() - t)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        laser_filter_node = LaserFilterNode()
        laser_filter_node.run()
    except rospy.ROSInterruptException:
        pass
