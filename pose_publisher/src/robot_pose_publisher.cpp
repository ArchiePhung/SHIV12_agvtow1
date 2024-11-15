/*
Author: HOANG VAN QUANG - BEE
Company: STI VietNam
Date: 21/10/2021
update: 15/02/2022
*/

// 8-3-2023 PHUCHOANG FIX

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

ros::Subscriber sub_odomNav;
ros::Publisher pub_poseRobot_nav;

// configuring parameters
std::string map_frame, base_frame;

// tf::TransformListener listener;
tf::StampedTransform transform;

int main(int argc, char ** argv)
{
	// initialize ROS and the node
	ros::init(argc, argv, "robotPose");
	ros::NodeHandle nh;
	// ros::NodeHandle nh("~");
	ros::Rate rate(10);

	map_frame = "/map"; // frame_map_nav350 frame_global_map
	base_frame = "/base_footprint";

	pub_poseRobot_nav = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);

	tf::TransformListener listener;
	// listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(1.0));

	ROS_INFO("Launch Right!");

	while (nh.ok())
	{
        try 
        {
            ros::Time now = ros::Time::now();
            listener.waitForTransform(map_frame, base_frame, now, ros::Duration(1.0)); // ros::Time(0)
            listener.lookupTransform(map_frame, base_frame, now, transform);

            // construct a pose message
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = base_frame;
            pose_stamped.header.stamp = ros::Time::now();

            pose_stamped.pose.orientation.x = transform.getRotation().getX();
            pose_stamped.pose.orientation.y = transform.getRotation().getY();
            pose_stamped.pose.orientation.z = transform.getRotation().getZ();
            pose_stamped.pose.orientation.w = transform.getRotation().getW();

            pose_stamped.pose.position.x = transform.getOrigin().getX();
            pose_stamped.pose.position.y = transform.getOrigin().getY();
            // pose_stamped.pose.position.z = transform.getOrigin().getZ();
            pose_stamped.pose.position.z = 0;

            pub_poseRobot_nav.publish(pose_stamped);

        } 
        catch (tf::TransformException &ex) 
        {
            // ROS_ERROR("%s",ex.what());
            ROS_INFO("TransformException ERROR!");
        } 
		// ROS_INFO("TransformException OK!");
		// ros::spinOnce();
		rate.sleep();
	}

	return EXIT_SUCCESS;
}
