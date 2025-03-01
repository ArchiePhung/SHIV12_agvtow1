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

//khai báo topic pose 
std::string topic_pose;

int main(int argc, char ** argv)
{
  
  ros::init(argc, argv, "robotPose");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  std::string map_frame, base_frame;
  double publish_frequency;
  bool is_stamped;
  ros::Publisher p_pub;

  nh_priv.param<std::string>("map_frame",map_frame,"/map");
  nh_priv.param<std::string>("base_frame",base_frame,"/base_footprint");
  nh_priv.param<std::string>("topic_pose",topic_pose,"robotPose_lidarLOC");

  nh_priv.param<double>("publish_frequency",publish_frequency,30);
  nh_priv.param<bool>("is_stamped", is_stamped, false);


  p_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_pose, 10);

  // create the listener
  tf::TransformListener listener;
  listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(1.0));

  ros::Rate rate(publish_frequency);
  while (nh.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);

      // construct a pose message
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = map_frame;
      pose_stamped.header.stamp = ros::Time::now();

      pose_stamped.pose.orientation.x = transform.getRotation().getX();
      pose_stamped.pose.orientation.y = transform.getRotation().getY();
      pose_stamped.pose.orientation.z = transform.getRotation().getZ();
      pose_stamped.pose.orientation.w = transform.getRotation().getW();

      pose_stamped.pose.position.x = transform.getOrigin().getX();
      pose_stamped.pose.position.y = transform.getOrigin().getY();
      pose_stamped.pose.position.z = transform.getOrigin().getZ();

    p_pub.publish(pose_stamped);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ROS_INFO("TransformException ERROR!");
    }

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
