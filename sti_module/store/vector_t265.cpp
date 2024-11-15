
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#define pi 3.141592653589793238462643383279502884
#define can2_2 0.7071067812

nav_msgs::Odometry odom_robot,odom_t265 ;

double a,b,z,w,x,y,d ;

void getOdom_t265(const nav_msgs::Odometry& odom)
{
    // odom_t265 = odom ; 
    x = odom.pose.pose.position.x; 
    y = odom.pose.pose.position.y; 
    z = odom.pose.pose.orientation.z;
    w = odom.pose.pose.orientation.w;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "vector_t265");
  ros::NodeHandle n;
  ros::Publisher pose1 = n.advertise<nav_msgs::Odometry>("odom_robot_via_t265", 50);
  ros::Subscriber sub = n.subscribe("/t265/odom/sample", 100, getOdom_t265);

  tf::TransformBroadcaster odom_broadcaster1, odom_broadcaster2, odom_broadcaster3;
  tf::TransformBroadcaster odom_broadcaster4, odom_broadcaster5, odom_broadcaster6;
  ros::Time current_time, last_time ;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ros::Rate r(200.0);
  while(n.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
    d = -0.265 ;
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans1;
    odom_trans1.header.stamp = current_time;
    odom_trans1.header.frame_id = "map_ao";
    odom_trans1.child_frame_id = "t265";

    odom_trans1.transform.translation.x = x ;
    odom_trans1.transform.translation.y = y ;
    odom_trans1.transform.translation.z = 0;
  
    odom_trans1.transform.rotation.x = 0;
    odom_trans1.transform.rotation.y = 0;
    odom_trans1.transform.rotation.z = z;
    odom_trans1.transform.rotation.w = w;
    //send the transform
    odom_broadcaster1.sendTransform(odom_trans1);


    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans2;
        odom_trans2.header.stamp = current_time;
        odom_trans2.header.frame_id = "t265";
        odom_trans2.child_frame_id = "robot";

        odom_trans2.transform.translation.x = d ;
        odom_trans2.transform.translation.y = 0 ;
        odom_trans2.transform.translation.z = 0;

        odom_trans2.transform.rotation.x = 0;
        odom_trans2.transform.rotation.y = 0;
        odom_trans2.transform.rotation.z = -can2_2;
        odom_trans2.transform.rotation.w = can2_2;
        odom_broadcaster2.sendTransform(odom_trans2);



    // aruco --> robot
    a = can2_2 * ( z + w );
    b = can2_2 * ( z - w );

    double x_rb, y_rb ;

    x_rb = (a*a*x - a*a*d - 2*a*b*y + b*b*x + b*b*a )/(a*a + b*b);
    y_rb = (2*a*b*x - a*b*d - b*b*y + a*a*y - a*a*b )/(a*a + b*b);

    geometry_msgs::TransformStamped odom_trans6;
    odom_trans6.header.stamp = current_time;
    odom_trans6.header.frame_id = "map_ao";
    odom_trans6.child_frame_id = "rb";

    odom_trans6.transform.translation.x = 0 ;
    odom_trans6.transform.translation.y = 0  ;
    odom_trans6.transform.translation.z = 0 ;

    odom_trans6.transform.rotation.x = 0;
    odom_trans6.transform.rotation.y = 0;
    odom_trans6.transform.rotation.z = b ;
    odom_trans6.transform.rotation.w = a ;

    odom_broadcaster6.sendTransform(odom_trans6);
    


    r.sleep();
     
  }
}
