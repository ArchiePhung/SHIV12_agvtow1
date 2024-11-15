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
geometry_msgs::PoseWithCovarianceStamped poseRb; 
bool sub4 = false ;
std::string tag_name, f_tag_name ;

// Toa do - Quatanion
    // map -> aruco
    geometry_msgs::Point p4;
    tf2::Quaternion q4 ,q4_n;

    geometry_msgs::Point p2;
    tf2::Quaternion q2 ,q2_n;
    

// call
    // 5hz
    void callSub4(const apriltag_ros::AprilTagDetectionArray& pose){ 
        getPosearuco = pose ; 
        // std::string tag   = "/tag_";
        // std::string f_tag = "/f_tag_";
        // std::string id = to_string(getPosearuco.detections[].id[]);
        // tag_name = tag + id ;
        // f_tag_name = f_tag + id ;
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
        ros::Publisher  pub1    = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("posePoseRobotFromAruco", 100);
        ros::Publisher  setpost = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 50);
    // TF
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
        // ROS_INFO("hihi");
        int id_aruco = 2;  //
        
        if (sub4 == true){  
            int sl_tag = getPosearuco.detections.size() ;
            if(sl_tag !=0 ){
                 std::string tag   = "/tag_";
                std::string f_tag = "/f_tag_";
                int id_aruco = getPosearuco.detections.at(0).id.at(0);
                std::string id = to_string(id_aruco);
                f_tag_name = f_tag + id ;
                tag_name = tag + id ;
            }       
           
            sub4 = false ;

         // map-> aruco
            
            //tag11
            p4.x  =   1.458850 ;
            p4.y  =   0.061912 ;
            p4.z  =   0.395693 ;
            q4[0] =   0.528474 ;
            q4[1] =   -0.497129 ;
            q4[2] =   -0.460255 ;
            q4[3] =   0.511609 ;

            odom_trans4.header.stamp = ros::Time::now();
            odom_trans4.header.frame_id = "/map";
            odom_trans4.child_frame_id =  "/f_tag_11";
            odom_trans4.transform.translation.x = p4.x  ;                                         
            odom_trans4.transform.translation.y = p4.y  ;
            odom_trans4.transform.translation.z = p4.z  ;
            odom_trans4.transform.rotation.x    = q4[0] ;
            odom_trans4.transform.rotation.y    = q4[1] ;
            odom_trans4.transform.rotation.z    = q4[2] ; 
            odom_trans4.transform.rotation.w    = q4[3] ; 
            odom_broadcaster4.sendTransform(odom_trans4);

            //tag10
            p2.x  = -11.694201 ;
            p2.y  = 2.524639 ;
            p2.z  = 0.404962 ;
            q2[0] = 0.720468 ;
            q2[1] = 0.019534 ;
            q2[2] = 0.009474 ;
            q2[3] = 0.693149 ;
            odom_trans5.header.stamp = ros::Time::now();
            odom_trans5.header.frame_id = "map";
            odom_trans5.child_frame_id =  "/f_tag_10";
            odom_trans5.transform.translation.x = p2.x  ;                                         
            odom_trans5.transform.translation.y = p2.y  ;
            odom_trans5.transform.translation.z = p2.z  ;
            odom_trans5.transform.rotation.x    = q2[0] ;
            odom_trans5.transform.rotation.y    = q2[1] ;
            odom_trans5.transform.rotation.z    = q2[2] ; 
            odom_trans5.transform.rotation.w    = q2[3] ; 
            odom_broadcaster5.sendTransform(odom_trans5);


         // aruco -> cam 
            try{
                listener3.lookupTransform(tag_name, "/camera_aruco", ros::Time(0), transform3);
                odom_trans3.header.stamp = ros::Time::now();
                odom_trans3.header.frame_id = f_tag_name;
                odom_trans3.child_frame_id =  "f_cam";
                odom_trans3.transform.translation.x = transform3.getOrigin().x() ;                                          
                odom_trans3.transform.translation.y = transform3.getOrigin().y() ;
                odom_trans3.transform.translation.z = transform3.getOrigin().z() ;
                odom_trans3.transform.rotation.x    = transform3.getRotation().x() ;
                odom_trans3.transform.rotation.y    = transform3.getRotation().y() ;
                odom_trans3.transform.rotation.z    = transform3.getRotation().z() ; 
                odom_trans3.transform.rotation.w    = transform3.getRotation().w() ; 
                odom_broadcaster3.sendTransform(odom_trans3);

            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }

         // cam -> robot ( base_footprint)
            try{
                listener2.lookupTransform("/camera_aruco", "/base_footprint", ros::Time(0), transform2);
                odom_trans2.header.stamp = ros::Time::now();
                odom_trans2.header.frame_id = "f_cam";
                odom_trans2.child_frame_id =  "f_robot";
                odom_trans2.transform.translation.x = transform2.getOrigin().x() ;                                          
                odom_trans2.transform.translation.y = transform2.getOrigin().y() ;
                odom_trans2.transform.translation.z = transform2.getOrigin().z() ;
                odom_trans2.transform.rotation.x    = transform2.getRotation().x() ;
                odom_trans2.transform.rotation.y    = transform2.getRotation().y() ;
                odom_trans2.transform.rotation.z    = transform2.getRotation().z() ; 
                odom_trans2.transform.rotation.w    = transform2.getRotation().w() ; 
                odom_broadcaster2.sendTransform(odom_trans2);

            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }

         // find map -> robot
            
            try{
                listener1.lookupTransform("/map", "/f_robot", ros::Time(0), transform1);
                odom_trans1.header.stamp = ros::Time::now();
                odom_trans1.header.frame_id = "map";
                odom_trans1.child_frame_id =  "f_rb";
                odom_trans1.transform.translation.x = transform1.getOrigin().x() ;                                          
                odom_trans1.transform.translation.y = transform1.getOrigin().y() ;
                odom_trans1.transform.translation.z = transform1.getOrigin().z() ;

                odom_trans1.transform.rotation.x    = transform1.getRotation().x() ;
                odom_trans1.transform.rotation.y    = transform1.getRotation().y() ;
                odom_trans1.transform.rotation.z    = transform1.getRotation().z() ; 
                odom_trans1.transform.rotation.w    = transform1.getRotation().w() ; 
                odom_broadcaster1.sendTransform(odom_trans1);

                poseRb.header.stamp = ros::Time::now();
                poseRb.header.frame_id = "map";
                poseRb.pose.pose.position.x = transform1.getOrigin().x() ;                                          
                poseRb.pose.pose.position.y = transform1.getOrigin().y() ;
                poseRb.pose.pose.position.z = 0; //transform1.getOrigin().z() ;
                poseRb.pose.pose.orientation.x    = transform1.getRotation().x() ;
                poseRb.pose.pose.orientation.y    = transform1.getRotation().y() ;
                poseRb.pose.pose.orientation.z    = transform1.getRotation().z() ; 
                poseRb.pose.pose.orientation.w    = transform1.getRotation().w() ; 
                pub1.publish(poseRb);

                // secs1 =ros::Time::now().toSec();
                // if ( secs1 - secs2 > 0.5) {
                //     poseRb.header.frame_id = "map";
                //     setpost.publish(poseRb);
                //     secs2 = ros::Time::now().toSec() ; 
                // }
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());

            }
        }
 
        r.sleep();
    }
}
