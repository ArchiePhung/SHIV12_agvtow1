
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define pi 3.141592653589793238462643383279502884
#define can2_2 0.7071067812
float px_robot,py_robot,goc_robot ; 
double roll,roll_ht,roll_set, pitch, yaw, yaw_ht,yaw_set, yaw_rad;

// geometry_msgs::PoseArray aruco_map ;
apriltag_ros::AprilTagDetectionArray getPosearuco, poseAruco; 
geometry_msgs::Pose aruco ; 

geometry_msgs::Pose robot_map; 
float tx, ty, tz, ox, oy, oz, ow ;
geometry_msgs::TransformStamped trans1 ;
bool enable_tf = false ;
bool sub4 = false ;

geometry_msgs::Pose aruco_map; 

// 5hz
void callSub4(const apriltag_ros::AprilTagDetectionArray& pose)
{ 
    getPosearuco = pose ; 
    sub4 = true ;
}

// 10hz
void callSub3(const geometry_msgs::Pose& pose3)
{
    robot_map = pose3 ;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    // subcriber
    ros::Subscriber subscribe4 = n.subscribe("/tag_detections", 1000, callSub4);
    ros::Subscriber subscribe3 = n.subscribe("/robot_pose", 1000, callSub3);
    // publisher
    ros::Publisher pose2 = n.advertise<geometry_msgs::Pose>("robotToMap", 50);

    //TF
    tf::TransformBroadcaster        odom_broadcaster1, odom_broadcaster2, odom_broadcaster3;
    geometry_msgs::TransformStamped odom_trans1      , odom_trans2      , odom_trans3;

    ros::Time current_time, last_time ;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(10.0);
    while(n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();
        int id_aruco = 2; 
        if (sub4 == true){         
            if (getPosearuco.detections[0].id[0] == id_aruco ){
                aruco = getPosearuco.detections[0].pose.pose.pose;
            } 
            sub4 = false ;
        }

        
        // map -> robot
        tf2::Quaternion q1(0,\
                           0,\
                           robot_map.orientation.z,\
                           robot_map.orientation.w );
        //robot - > cam
        tf2::Quaternion q2((-1 * can2_2 ),\
                           0,\
                           0,\
                           can2_2);
        // cam -> aruco1
        tf2::Quaternion qAr1(aruco.orientation.x,\
                             aruco.orientation.y,\
                             aruco.orientation.z,\
                             aruco.orientation.w );

        // ROS_INFO("qAr1[0]= %f",qAr1[0]);
        // ROS_INFO("aruco[x]= %f",aruco.orientation.x);

        tf2::Quaternion q12 ;
        q12= q1*q2;

        tf2::Quaternion q12Ar1, q12Ar2, q12Ar3, q12Ar4 ;
        q12Ar1 = q12*qAr1; 

        // goc cam -> aruco 1
        double rollAr1, pitchAr1, yawAr1 ;
        tf2::Matrix3x3 mAr1(qAr1);
        mAr1.getRPY(rollAr1, pitchAr1, yawAr1); // rad
 
        // goc map->cam
        double rollMc, pitchMc, yawArMc ;
        tf2::Matrix3x3 mMc(q12);
        mMc.getRPY(rollMc, pitchMc, yawArMc); // rad

        
        // x' = x*cos(alpha) - y*sin(alpha)
        // y' = x*sin(alpha) + y*cos(alpha)
        double x_Ar1 = robot_map.position.x + ((0 + aruco.position.x)*cos(yawArMc) - (0.265 + aruco.position.z)*sin(yawArMc))  ;
        double y_Ar1 = robot_map.position.y + ((0 + aruco.position.x)*sin(yawArMc) + (0.265 + aruco.position.z)*cos(yawArMc))  ;
        double z_Ar1 = 0.27 + aruco.position.y*cos(rollAr1); 
        
        // map -> aruco 
            odom_trans1.header.stamp = current_time;
            odom_trans1.header.frame_id = "map";
            odom_trans1.child_frame_id =  "aru_1";
            odom_trans1.transform.translation.x = x_Ar1 ;                                          
            odom_trans1.transform.translation.y = y_Ar1 ;
            odom_trans1.transform.translation.z = z_Ar1;
            odom_trans1.transform.rotation.x = q12Ar1[0];
            odom_trans1.transform.rotation.y = q12Ar1[1];
            odom_trans1.transform.rotation.z = q12Ar1[2]; 
            odom_trans1.transform.rotation.w = q12Ar1[3]; 
            odom_broadcaster1.sendTransform(odom_trans1);

        ROS_INFO("map->aruco %d : x= %f , y= %f , z= %f ",id_aruco, x_Ar1, y_Ar1, z_Ar1 );
        ROS_INFO("map->aruco %d : rx= %f , ry= %f , rz= %f , rw= %f", id_aruco,q12Ar1[0],q12Ar1[1],q12Ar1[2],q12Ar1[3]);
        ROS_INFO("poseRB : x= %f , y= %f , rz= %f , rw= %f \n", robot_map.position.x,robot_map.position.y , robot_map.orientation.z,robot_map.orientation.w);

     
        r.sleep();
        
    }
}
