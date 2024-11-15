
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#define pi 3.141592653589793238462643383279502884
float px_robot,py_robot,goc_robot ; 
double roll,roll_ht,roll_set, pitch, yaw, yaw_ht,yaw_set, yaw_rad;

geometry_msgs::PoseStamped  pose_aruco , pose_camera ;

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Publisher pose1 = n.advertise<geometry_msgs::PoseStamped >("pose_aruco", 50);
  ros::Publisher pose2 = n.advertise<geometry_msgs::PoseStamped >("pose_camera", 50);

  tf::TransformBroadcaster odom_broadcaster1, odom_broadcaster2, odom_broadcaster3;
  tf::TransformBroadcaster odom_broadcaster4, odom_broadcaster5, odom_broadcaster6;
  ros::Time current_time, last_time ;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ros::Rate r(200.0);
  
  yaw_rad = 2;
  px_robot = 1 ; 
  py_robot = 1 ;

  while(n.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();

    // ROS_INFO("yaw= %f", yaw );

    // geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(yaw_rad);



    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans1;
    odom_trans1.header.stamp = current_time;
    odom_trans1.header.frame_id = "map";
    odom_trans1.child_frame_id = "aruco";

    odom_trans1.transform.translation.x = -0.143081694841;
    odom_trans1.transform.translation.y = 1.18045175076;
    odom_trans1.transform.translation.z = 0.352265805006;
  
    odom_trans1.transform.rotation.x = 0.7140203555;
    odom_trans1.transform.rotation.y = 0.0132851026291;
    odom_trans1.transform.rotation.z = 0.00583272793744;
    odom_trans1.transform.rotation.w = 0.699974571598;
    //send the transform
    odom_broadcaster1.sendTransform(odom_trans1);



    //first, we'll publish the transform over tf
    // geometry_msgs::TransformStamped odom_trans2;
    // odom_trans2.header.stamp = current_time;
    // odom_trans2.header.frame_id = "robot";
    // odom_trans2.child_frame_id = "camera_haha";

    // odom_trans2.transform.translation.x = 0;
    // odom_trans2.transform.translation.y = 0.265 ;
    // odom_trans2.transform.translation.z = 0.27;

    // odom_trans2.transform.rotation.x = -0.707;
    // odom_trans2.transform.rotation.y = 0;
    // odom_trans2.transform.rotation.z = 0;
    // odom_trans2.transform.rotation.w = 0.707;
    
    // odom_broadcaster2.sendTransform(odom_trans2);


    //camera_haha - > aruco1

    float tx_cam_ar = -0.172740443526;
    float ty_cam_ar = -0.0940858069139;
    float tz_cam_ar = 0.912485275883;
    float rx_cam_ar = 0.999360122361;
    float ry_cam_ar = 0.00350564132015;
    float rz_cam_ar = 0.0355005930305;
    float rw_cam_ar = 0.00260080901242;
    // geometry_msgs::TransformStamped odom_trans3;
    // odom_trans3.header.stamp = current_time;
    // odom_trans3.header.frame_id = "camera_haha";
    // odom_trans3.child_frame_id = "aruco1";

    // odom_trans3.transform.translation.x = tx_cam_ar;
    // odom_trans3.transform.translation.y = ty_cam_ar;
    // odom_trans3.transform.translation.z = tz_cam_ar;

    // odom_trans3.transform.rotation.x = rx_cam_ar;
    // odom_trans3.transform.rotation.y = ry_cam_ar;
    // odom_trans3.transform.rotation.z = rz_cam_ar;
    // odom_trans3.transform.rotation.w = rw_cam_ar;
    // odom_broadcaster3.sendTransform(odom_trans3);


    //aruco -> camera_haha
    geometry_msgs::TransformStamped odom_trans4;
    odom_trans4.header.stamp = current_time;
    odom_trans4.header.frame_id = "aruco";
    odom_trans4.child_frame_id = "camera";

    odom_trans4.transform.translation.x = (-1) * tx_cam_ar;
    odom_trans4.transform.translation.y = ty_cam_ar;
    odom_trans4.transform.translation.z = tz_cam_ar;

    odom_trans4.transform.rotation.x = rx_cam_ar;
    odom_trans4.transform.rotation.y = ry_cam_ar;
    odom_trans4.transform.rotation.z = rz_cam_ar;
    odom_trans4.transform.rotation.w = rw_cam_ar;
    odom_broadcaster4.sendTransform(odom_trans4);

    // cam --> robot
    // geometry_msgs::TransformStamped odom_trans5;
    // odom_trans5.header.stamp = current_time;
    // odom_trans5.header.frame_id = "camera";
    // odom_trans5.child_frame_id = "robot1";

    // odom_trans5.transform.translation.x = 0;
    // odom_trans5.transform.translation.y = 0.27 ;
    // odom_trans5.transform.translation.z = -0.265;

    // odom_trans5.transform.rotation.x = 0.707;
    // odom_trans5.transform.rotation.y = 0;
    // odom_trans5.transform.rotation.z = 0;
    // odom_trans5.transform.rotation.w = 0.707;

    // odom_broadcaster5.sendTransform(odom_trans5);

     // aruco --> robot
    geometry_msgs::TransformStamped odom_trans6;
    odom_trans6.header.stamp = current_time;
    odom_trans6.header.frame_id = "aruco";
    odom_trans6.child_frame_id = "robot2";

    odom_trans6.transform.translation.x = 0 - tx_cam_ar ;
    odom_trans6.transform.translation.y = -0.27 + ty_cam_ar ;
    odom_trans6.transform.translation.z = 0.265 + tz_cam_ar;

    odom_trans6.transform.rotation.x = -0.707;
    odom_trans6.transform.rotation.y = 0;
    odom_trans6.transform.rotation.z = 0;
    odom_trans6.transform.rotation.w = 0.707;

    odom_broadcaster6.sendTransform(odom_trans6);
    


    r.sleep();
     
  }
}
