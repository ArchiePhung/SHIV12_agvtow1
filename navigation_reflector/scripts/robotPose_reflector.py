#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped
from scipy.spatial.transform import Rotation as R
from collections import deque

class TransformFilter:
    def __init__(self):
        rospy.init_node('tf_median_filter', anonymous=True)
        
        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publisher for the filtered pose
        self.pose_pub = rospy.Publisher('/robotPose_reflector', PoseStamped, queue_size=10)
        
        # Deque to store last N transforms (for filtering)
        self.transform_history = deque(maxlen=5)  # Adjust window size as needed
        
        # Transform details
        self.source_frame = 'map'
        self.target_frame = 'base_footprintRef'
        
        # Frequency of update
        self.rate = rospy.Rate(20)  # 10 Hz update rate

    def get_transform(self):
        try:
            # Lookup the latest transformation from source_frame to target_frame
            transform = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, rospy.Time(0), rospy.Duration(1.0))
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
            return None

    def unwrap_angle(self, angle):
        """
        Unwraps the angle to prevent jumps between -180 and 180 degrees.
        """
        return np.unwrap([angle])[0]

    def filter_median(self):
        # Get the translation and rotation components of each transform in history
        translations = np.array([[t.transform.translation.x, t.transform.translation.y, t.transform.translation.z] for t in self.transform_history])
        rotations = np.array([[t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w] for t in self.transform_history])

        # Convert quaternion to Euler angles for filtering
        euler_angles = [R.from_quat(rot).as_euler('xyz', degrees=True) for rot in rotations]
        
        # Unwrap the yaw angle to avoid issues between 179 and -179 degrees
        yaw_angles = [self.unwrap_angle(angles[2]) for angles in euler_angles]  # Yaw is the third angle (z-axis rotation)
        
        # Compute the median of translations and yaw (rotation)
        median_translation = np.median(translations, axis=0)
        median_yaw = np.median(yaw_angles)
        
        # Convert back to quaternion from median Euler angles
        median_rotation = R.from_euler('xyz', [0, 0, median_yaw], degrees=True).as_quat()

        # Create a new TransformStamped with the median values
        median_transform = TransformStamped()
        median_transform.header.stamp = rospy.Time.now()
        median_transform.header.frame_id = self.source_frame
        median_transform.child_frame_id = self.target_frame
        median_transform.transform.translation.x = median_translation[0]
        median_transform.transform.translation.y = median_translation[1]
        median_transform.transform.translation.z = median_translation[2]
        median_transform.transform.rotation.x = median_rotation[0]
        median_transform.transform.rotation.y = median_rotation[1]
        median_transform.transform.rotation.z = median_rotation[2]
        median_transform.transform.rotation.w = median_rotation[3]

        return median_transform

    def publish_pose(self, transform):
        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.source_frame
        
        # Copy translation to pose
        pose_msg.pose.position.x = transform.transform.translation.x
        pose_msg.pose.position.y = transform.transform.translation.y
        pose_msg.pose.position.z = transform.transform.translation.z
        
        # Copy rotation to pose
        pose_msg.pose.orientation.x = transform.transform.rotation.x
        pose_msg.pose.orientation.y = transform.transform.rotation.y
        pose_msg.pose.orientation.z = transform.transform.rotation.z
        pose_msg.pose.orientation.w = transform.transform.rotation.w

        # Publish the pose
        self.pose_pub.publish(pose_msg)

    def run(self):
        while not rospy.is_shutdown():
            # Get the latest transform
            transform = self.get_transform()
            if transform:
                # Add to history
                self.transform_history.append(transform)
                
                # If history is full, filter and publish the median
                if len(self.transform_history) == self.transform_history.maxlen:
                    median_transform = self.filter_median()
                    # rospy.loginfo(f"Median Transform: {median_transform}")
                    
                    # Publish the filtered transform as a Pose
                    self.publish_pose(median_transform)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        tf_filter = TransformFilter()
        tf_filter.run()
    except rospy.ROSInterruptException:
        pass
