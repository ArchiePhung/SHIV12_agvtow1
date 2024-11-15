#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <string>
using namespace std;

#define pi 3.141592653589793238462643383279502884
#define can2_2 0.7071067812
geometry_msgs::PoseWithCovarianceStamped pose_estimaste ;
bool sub1 = false ;
std::string tag_name, f_tag_name ;
// 5hz
void callSub(const geometry_msgs::PoseWithCovarianceStamped& pose)
{ 
    pose_estimaste = pose ; 
    sub1 = true ;
}
// global_localization : bat uoc tinh toan ban do 



int main(int argc, char** argv){
    // init
        ros::init(argc, argv, "getPoseArucoInMap");
        ros::NodeHandle n;
        ros::Time current_time, last_vel_time;
        ros::Rate r(100.0);

    // subcriber
        ros::Subscriber subscribe1 = n.subscribe("/amcl_pose", 1000, callSub);
    // TF
        tf::TransformBroadcaster        broadcaster;
        geometry_msgs::TransformStamped trans      ;

    while(n.ok()){
        ros::spinOnce();
        if(sub1 == true){

            trans.header.stamp            = ros::Time::now();
            trans.header.frame_id         = "map";
            trans.child_frame_id          = "robot_estimaste";
            trans.transform.translation.x = pose_estimaste.pose.pose.position.x ;                                  
            trans.transform.translation.y = pose_estimaste.pose.pose.position.y ; 
            trans.transform.translation.z = pose_estimaste.pose.pose.position.z ; 
            trans.transform.rotation.x    = pose_estimaste.pose.pose.orientation.x ; 
            trans.transform.rotation.y    = pose_estimaste.pose.pose.orientation.y ; 
            trans.transform.rotation.z    = pose_estimaste.pose.pose.orientation.z ; 
            trans.transform.rotation.w    = pose_estimaste.pose.pose.orientation.w ;         
            broadcaster.sendTransform(trans);

            sub1 = false ;
        }



        r.sleep();
    }
}
