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
apriltag_ros::AprilTagDetectionArray getPosearuco; 
geometry_msgs::Pose aruco ; 
geometry_msgs::Pose robot_map; 
bool sub4 = false ;
std::string tag_name, f_tag_name ;
// 5hz
void callSub4(const apriltag_ros::AprilTagDetectionArray& pose)
{ 
    getPosearuco = pose ; 
    sub4 = true ;
}


// Toa do - Quatanion
    // map -> robot
    geometry_msgs::Point p1;
    tf2::Quaternion q1 ;
    // robot -> cam 
    geometry_msgs::Point p2;
    tf2::Quaternion q21((-1 * can2_2 ),\
                         0,\
                         0,\
                         can2_2);
    tf2::Quaternion q22(0,\
                        0,\
                        (-1 * can2_2 ),\
                        can2_2);
    tf2::Quaternion q2 ;
    // cam -> aruco  
    geometry_msgs::Point p3;
    tf2::Quaternion q3 ;
    // map -> aruco
    geometry_msgs::Point p4;
    tf2::Quaternion q4 ;
    // map -> camera
    geometry_msgs::Point p5;
    tf2::Quaternion q5 ;


int main(int argc, char** argv){
    // init
        ros::init(argc, argv, "getPoseArucoInMap");
        ros::NodeHandle n;
        ros::Time current_time, last_vel_time;
        ros::Rate r(100.0);

    // subcriber
        ros::Subscriber subscribe4 = n.subscribe("/tag_detections", 1000, callSub4);
    // TF
        tf::TransformBroadcaster        odom_broadcaster1;
        geometry_msgs::TransformStamped odom_trans1      ;

        tf::TransformListener listener;
        tf::StampedTransform transform;
        listener.waitForTransform("/map", tag_name, ros::Time(), ros::Duration(1.0));


    while(n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();
        int id_aruco = 2; 
        if (sub4 == true){         
            if (getPosearuco.detections[0].id[0] == id_aruco ){
                std::string tag   = "/tag_";
                std::string f_tag = "/f_tag_";
                std::string id = to_string(id_aruco);
                tag_name = tag + id ;
                f_tag_name = f_tag + id ;
                // ROS_INFO("tag_name = %s",tag_name.c_str());
            } 
            sub4 = false ;
        


        // find map -> aru    
           
            try{
            listener.lookupTransform("/map", tag_name,ros::Time(0), transform);

                odom_trans1.header.stamp = ros::Time::now();
                odom_trans1.header.frame_id = "map";
                odom_trans1.child_frame_id =  f_tag_name;
                odom_trans1.transform.translation.x = transform.getOrigin().x() ;                                            
                odom_trans1.transform.translation.y = transform.getOrigin().y() ;
                odom_trans1.transform.translation.z = transform.getOrigin().z() ;
                odom_trans1.transform.rotation.x    = transform.getRotation().x() ;
                odom_trans1.transform.rotation.y    = transform.getRotation().y() ;
                odom_trans1.transform.rotation.z    = transform.getRotation().z() ; 
                odom_trans1.transform.rotation.w    = transform.getRotation().w() ;                
                odom_broadcaster1.sendTransform(odom_trans1);

                ROS_INFO("map -> %s : x= %f ,y= %f ,z= %f ",tag_name.c_str(),transform.getOrigin().x(),\
                                                                               transform.getOrigin().y(),\
                                                                               transform.getOrigin().z());
                ROS_INFO("map -> %s : r_x= %f ,r_y= %f ,r_z= %f ,r_w= %f \n",tag_name.c_str(),\
                                                                             transform.getRotation().x(),\
                                                                             transform.getRotation().y(),\
                                                                             transform.getRotation().z(),\
                                                                             transform.getRotation().w());
                
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
                    
        }

        r.sleep();
    }
}
