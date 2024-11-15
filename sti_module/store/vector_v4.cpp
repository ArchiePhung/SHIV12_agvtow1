
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <fiducial_msgs/FiducialTransformArray.h>
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
geometry_msgs::Pose aruco[4] ; 
char id_aruco[4] = {1,20,3,4};

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

// void 

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    // subcriber
    // ros::Subscriber sub1 = n.subscribe("/rtabmap/landmarks", 1000, callSub1);
    // ros::Subscriber sub2 = n.subscribe("/fiducial_transforms", 100, callSub2);
    ros::Subscriber subscribe4 = n.subscribe("/tag_detections", 1000, callSub4);
    ros::Subscriber subscribe3 = n.subscribe("/robot_pose", 1000, callSub3);
    // publisher
    ros::Publisher pose2 = n.advertise<geometry_msgs::Pose>("robotToMap", 50);

    //TF
    tf::TransformBroadcaster        odom_broadcaster1, odom_broadcaster2, odom_broadcaster3;
    geometry_msgs::TransformStamped odom_trans1      , odom_trans2      , odom_trans3;
    tf::TransformBroadcaster        odom_broadcaster4, odom_broadcaster5, odom_broadcaster6;
    geometry_msgs::TransformStamped odom_trans4      , odom_trans5      , odom_trans6;
    tf::TransformBroadcaster        odom_broadcaster7, odom_broadcaster8, odom_broadcaster9;
    geometry_msgs::TransformStamped odom_trans7      , odom_trans8      , odom_trans9;

    ros::Time current_time, last_time ;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(10.0);
    while(n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();

        if (sub4 == true){
            for (int i=0; i<sizeof(id_aruco); i++){
                if (getPosearuco.detections[0].id[0] == id_aruco[i] ){
                    aruco[i] = getPosearuco.detections[0].pose.pose.pose;
                    ROS_INFO("id = %d \n",id_aruco[i] );
                } 
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
        tf2::Quaternion qAr1(aruco[0].orientation.x,\
                             aruco[0].orientation.y,\
                             aruco[0].orientation.z,\
                             aruco[0].orientation.w );
        // cam -> aruco20
        tf2::Quaternion qAr20(aruco[1].orientation.x,\
                              aruco[1].orientation.y,\
                              aruco[1].orientation.z,\
                              aruco[1].orientation.w );
        // cam -> aruco3
        tf2::Quaternion qAr3(aruco[2].orientation.x,\
                             aruco[2].orientation.y,\
                             aruco[2].orientation.z,\
                             aruco[2].orientation.w );
        // cam -> aruco4
        tf2::Quaternion qAr4(aruco[3].orientation.x,\
                             aruco[3].orientation.y,\
                             aruco[3].orientation.z,\
                             aruco[3].orientation.w );

        tf2::Quaternion q12 ;
        q12= q1*q2;

        tf2::Quaternion q12Ar1, q12Ar2, q12Ar3, q12Ar4 ;
        q12Ar1 = q1*q2*qAr1; 
        q12Ar2 = q1*q2*qAr2;
        q12Ar3 = q1*q2*qAr3;
        q12Ar4 = q1*q2*qAr4;

        // goc cam -> aruco 1
        double rollAr1, pitchAr1, yawAr1 ;
        tf::Matrix3x3 mAr1(qAr1);
        mAr1.getRPY(rollAr1, pitchAr1, yawAr1); // rad
        // goc cam -> aruco 2
        double rollAr2, pitchAr2, yawAr2 ;
        tf::Matrix3x3 mAr2(qAr2);
        mAr2.getRPY(rollAr12, pitchAr2, yawAr2); // rad
        // goc cam -> aruco 3
        double rollAr3, pitchAr3, yawAr3 ;
        tf::Matrix3x3 mAr3(qAr3);
        mAr3.getRPY(rollAr3, pitchAr3, yawAr3); // rad
        // goc cam -> aruco 4
        double rollAr4, pitchAr4, yawAr4 ;
        tf::Matrix3x3 mAr4(qAr4);
        mAr4.getRPY(rollAr4, pitchAr4, yawAr4); // rad
        // goc map->cam
        double rollMc, pitchMc, yawArMc ;
        tf::Matrix3x3 mMc(q12);
        mMc.getRPY(rollMc, pitchMc, yawArMc); // rad

        


        // x' = x*cos(alpha) - y*sin(alpha)
        // y' = x*sin(alpha) + y*cos(alpha)
        double x_Ar1 = robot_map.position.x + ((0 + poseAruco.detections[1].pose.pose.pose.position.x)*cos(yaw12) - (0.265 + poseAruco.detections[1].pose.pose.pose.position.z)*sin(yaw12))  ;
        double y_Ar1 = robot_map.position.y + ((0 + poseAruco.detections[1].pose.pose.pose.position.x)*sin(yaw12) + (0.265 + poseAruco.detections[1].pose.pose.pose.position.z)*cos(yaw12))  ;
        double z_Ar1 = 0.27 + cam_aruco.position.y*cos(roll3); 
        ROS_INFO("map> aruco : x= %f , y= %f , z= %f \n", x_Ar1, y_Ar1, z_Ar1 );
        // map -> aruco 
            odom_trans1.header.stamp = current_time;
            odom_trans1.header.frame_id = "map";
            odom_trans1.child_frame_id =  "aru_1";
            odom_trans1.transform.translation.x = x_Ar1 ;                                          
            odom_trans1.transform.translation.y = y_Ar1 ;
            odom_trans1.transform.translation.z = z_Ar1;
            odom_trans1.transform.rotation.x = x123;
            odom_trans1.transform.rotation.y = y123;
            odom_trans1.transform.rotation.z = z123; 
            odom_trans1.transform.rotation.w = w123; 
            odom_broadcaster2.sendTransform(odom_trans2);

        // int e =0 ; 
        // if (enable_tf == true){
        // if (e ==1 ){
        //     // tinh quatanion aruco -> map 

        //     double w1, z1, y1, x1, w2, x2, y2, z2, w3, x3, y3, z3 ;
        //     double w12, x12, y12, z12, w123, x123, y123, z123;
        //     double w23, x23, y23, z23 ;
        //     // map -> robot
        //     w1 = robot_map.orientation.w ; 
        //     z1 = robot_map.orientation.z ; 
        //     y1 = 0 ;
        //     x1 = 0 ;
        //     //robot - > cam
        //     w2 = can2_2 ;
        //     z2 = 0 ;
        //     y2 = 0 ;
        //     x2 = -1 * can2_2 ;
        //     // cam -> aruco
        //     // w3 = cam_aruco.orientation.w ;
        //     // x3 = cam_aruco.orientation.x ;
        //     // y3 = cam_aruco.orientation.y ;
        //     // z3 = cam_aruco.orientation.z ;
        //     w3 = poseAruco.detections[1].pose.pose.pose.orientation.w ;
        //     x3 = poseAruco.detections[1].pose.pose.pose.orientation.x ;
        //     y3 = poseAruco.detections[1].pose.pose.pose.orientation.y ;
        //     z3 = poseAruco.detections[1].pose.pose.pose.orientation.z ;

        //     // ==> map -> cam
        //     w12 = w1*w2 ; 
        //     x12 = w1*x2 ; 
        //     y12 = z1*x2 ;
        //     z12 = w2*z1 ; 
        //     // ROS_INFO("w12= %f , x12= %f , y12= %f , z12= %f \n", w12,x12,y12,z12 );

        //     // ==> robot -> aruco 
        //     w23 = w2*w3 - x2*x3 - y2*y3 - z2*z3 ;
        //     x23 = w2*x3 + x2*w3 + y2*z3 - z2*y3 ;
        //     y23 = w2*y3 - x2*z3 + y2*w3 + z2*x3 ;
        //     z23 = w2*z3 + x2*y3 - y2*x3 + z2*w3 ;
        //     // ROS_INFO("w23= %f , x23= %f , y23= %f , z23= %f \n", w23,x23,y23,z23 );

        //     // ==> map -> aruco 
        //     w123 = w12*w3 - x12*x3 - y12*y3 - z12*z3 ;
        //     x123 = w12*x3 + x12*w3 + y12*z3 - z12*y3 ;
        //     y123 = w12*y3 - x12*z3 + y12*w3 + z12*x3 ;
        //     z123 = w12*z3 + x12*y3 - y12*x3 + z12*w3 ;
        //     // ROS_INFO("w123= %f , x123= %f , y123= %f , z123= %f \n", w123,x123,y123,z123 );

        //     // goc map - cam
        //     double roll12, pitch12, yaw12 ;
        //     tf::Quaternion q12(x12,y12,z12,w12);
        //     tf::Matrix3x3 m12(q12);
        //     m12.getRPY(roll12, pitch12, yaw12); // rad
        //     //  ROS_INFO("roll12= %f , pitch12= %f , yaw12= %f \n", roll12, pitch12, yaw12);

        //     // goc map->aruco 
        //     double roll123, pitch123, yaw123 ;
        //     tf::Quaternion q123(x123,y123,z123,w123);
        //     tf::Matrix3x3 m123(q123);
        //     m123.getRPY(roll123, pitch123, yaw123); // rad
        //     // ROS_INFO("map->aruco : roll123= %f , pitch123= %f , yaw123= %f \n", roll123, pitch123, yaw123 );



        //     // goc robot- aruco 
        //     double roll23, pitch23, yaw23 ;
        //     tf::Quaternion q23(x23,y23,z23,w23);
        //     tf::Matrix3x3 m23(q23);
        //     m23.getRPY(roll23, pitch23, yaw23); // rad
        //     // ROS_INFO("robot->aruco: roll23= %f , pitch23= %f , yaw23= %f \n", roll23, pitch23, yaw23 );

        //     // goc robot- map 
        //     double roll1, pitch1, yaw1 ;
        //     tf::Quaternion q1(x1,y1,z1,w1);
        //     tf::Matrix3x3 m1(q1);
        //     m1.getRPY(roll1, pitch1, yaw1); // rad
        //     // ROS_INFO("roll1= %f , pitch1= %f , yaw1= %f \n", roll1, pitch1, yaw1 );

        //     // x' = x*cos(alpha) - y*sin(alpha)
        //     // y' = x*sin(alpha) + y*cos(alpha)
        //     double xx2 = robot_map.position.x + ((0 + poseAruco.detections[1].pose.pose.pose.position.x)*cos(yaw12) - (0.265 + poseAruco.detections[1].pose.pose.pose.position.z)*sin(yaw12))  ;
        //     double yy2 = robot_map.position.y + ((0 + poseAruco.detections[1].pose.pose.pose.position.x)*sin(yaw12) + (0.265 + poseAruco.detections[1].pose.pose.pose.position.z)*cos(yaw12))  ;
        //     double zz2 = 0.27 + cam_aruco.position.y*cos(roll3); 
        //     ROS_INFO("map> aruco : x= %f , y= %f , z= %f \n", xx2, yy2, zz2 );
        //     // map -> aruco 
        //         odom_trans2.header.stamp = current_time;
        //         odom_trans2.header.frame_id = "map";
        //         odom_trans2.child_frame_id =  "aru";
        //         odom_trans2.transform.translation.x = xx2 ;                                          
        //         odom_trans2.transform.translation.y = yy2 ;
        //         odom_trans2.transform.translation.z = zz2;
        //         odom_trans2.transform.rotation.x = x123;
        //         odom_trans2.transform.rotation.y = y123;
        //         odom_trans2.transform.rotation.z = z123; 
        //         odom_trans2.transform.rotation.w = w123; 
        //         odom_broadcaster2.sendTransform(odom_trans2);
        // }

        r.sleep();
        
    }
}
