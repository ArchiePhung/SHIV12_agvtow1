#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <string>
using namespace std;

#define pi 3.141592653589793238462643383279502884
#define can2_2 0.7071067812
apriltag_ros::AprilTagDetectionArray getPosearuco, getPosearuco1,getPosearuco2,getPosearuco3 ; 
apriltag_ros::AprilTagDetection      PoseAr1,PoseAr2,PoseAr3; 
geometry_msgs::PoseWithCovarianceStamped poseRb; 
bool sub4 = false ;
std::string tag_name, f_tag_name ;

// Toa do - Quatanion
    // map -> aruco
    geometry_msgs::Point p4;
    tf2::Quaternion q4 ,q4_n;
    

// call
    // 5hz
    void callSub4(const apriltag_ros::AprilTagDetectionArray& po){ 
        std::vector<apriltag_ros::AprilTagDetection> detec;
        int size = po.detections.size() ;
        
        ROS_INFO("size= %d",size);

        sub4 = true ;
    }


int main(int argc, char** argv){
    // init
        ros::init(argc, argv, "getPoseRobotFromAruco");
        ros::NodeHandle n;
        ros::Time current_time, last_vel_time;
        ros::Rate r(100.0);

    // sub - pub
        ros::Subscriber sub1    = n.subscribe("/tag_detections", 100, callSub4);
        // ros::Publisher  pub1    = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("posePoseRobotFromAruco", 100);
        // ros::Publisher  setpost = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 50);
    //TF
        tf::TransformBroadcaster        odom_broadcaster1, odom_broadcaster2, odom_broadcaster3;
        geometry_msgs::TransformStamped odom_trans1      , odom_trans2      , odom_trans3;
        tf::TransformBroadcaster        odom_broadcaster4, odom_broadcaster5, odom_broadcaster6;
        geometry_msgs::TransformStamped odom_trans4      , odom_trans5      , odom_trans6;

        tf::TransformListener listener1,listener2,listener3,listener4;
        tf::StampedTransform transform1,transform2,transform3,transform4;
        // listener1.waitForTransform("/map", tag_name, ros::Time(), ros::Duration(1.0));
    // sec
        double secs1 =ros::Time::now().toSec();
        double secs2 = ros::Time::now().toSec() ;

    while(n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();
        

 
        r.sleep();
    }
}
