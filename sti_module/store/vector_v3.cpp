
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#define pi 3.141592653589793238462643383279502884
#define can2_2 0.7071067812
float px_robot,py_robot,goc_robot ; 
double roll,roll_ht,roll_set, pitch, yaw, yaw_ht,yaw_set, yaw_rad;

// geometry_msgs::PoseArray aruco_map ;
// fiducial_msgs::FiducialTransformArray cam_aruco; 
geometry_msgs::Pose cam_aruco; 
geometry_msgs::Pose robot_map; 
float tx, ty, tz, ox, oy, oz, ow ;
geometry_msgs::TransformStamped trans1 ;
bool enable_tf ;

geometry_msgs::Pose aruco_map; 

// 5hz
void callSub4(const apriltag_ros::AprilTagDetectionArray& pose2)
{
    for (int i=0;i<10; i++){
        if ( pose2.detections[0].id[0] == 4 ){
            cam_aruco = pose2.detections[0].pose.pose.pose ;
            enable_tf = true ;
            // ROS_INFO("px=%f",cam_aruco.position.x );
        }
    }
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
    // ros::Subscriber sub1 = n.subscribe("/rtabmap/landmarks", 1000, callSub1);
    // ros::Subscriber sub2 = n.subscribe("/fiducial_transforms", 100, callSub2);
    ros::Subscriber sub4 = n.subscribe("/tag_detections", 100, callSub4);
    ros::Subscriber sub3 = n.subscribe("/robot_pose", 1000, callSub3);
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
        // TF cam -> aruco 
            // odom_trans1.header.stamp = current_time;
            // odom_trans1.header.frame_id = "rgb_camera_link";
            // odom_trans1.child_frame_id =  "aruco";
            // odom_cam_aruco.position = cam_aruco.position ;
            // odom_cam_aruco.orientation    = cam_aruco.orientation ; 
            // if ( enable_tf ==true ) odom_broadcaster1.sendTransform(odom_trans1);

        // robot - > rbg_cam 
            odom_trans3.header.stamp = current_time;
            odom_trans3.header.frame_id = "base_footprint";
            odom_trans3.child_frame_id =  "rbg_cam";
            odom_trans3.transform.translation.x = 0.0; //0.03;
            odom_trans3.transform.translation.y = 0.265 ;
            odom_trans3.transform.translation.z = 0.27;
            odom_trans3.transform.rotation.x = -0.707;
            odom_trans3.transform.rotation.y = 0;
            odom_trans3.transform.rotation.z = 0;
            odom_trans3.transform.rotation.w = 0.707;
            // odom_broadcaster3.sendTransform(odom_trans3);

        // tinh quatanion aruco -> map 

        double w1, z1, y1, x1, w2, x2, y2, z2, w3, x3, y3, z3 ;
        double w12, x12, y12, z12, w123, x123, y123, z123;
        double w23, x23, y23, z23 ;
        // map -> robot
        w1 = robot_map.orientation.w ; 
        z1 = robot_map.orientation.z ; 
        y1 = 0 ;
        x1 = 0 ;
        //robot - > cam
        w2 = can2_2 ;
        z2 = 0 ;
        y2 = 0 ;
        x2 = -1 * can2_2 ;
        // cam -> aruco
        w3 = cam_aruco.orientation.w ;
        x3 = cam_aruco.orientation.x ;
        y3 = cam_aruco.orientation.y ;
        z3 = cam_aruco.orientation.z ;

        // ==> map -> cam
        w12 = w1*w2 ; 
        x12 = w1*x2 ; 
        y12 = z1*x2 ;
        z12 = w2*z1 ; 
        // ROS_INFO("w12= %f , x12= %f , y12= %f , z12= %f \n", w12,x12,y12,z12 );

        // ==> robot -> aruco 
        w23 = w2*w3 - x2*x3 - y2*y3 - z2*z3 ;
        x23 = w2*x3 + x2*w3 + y2*z3 - z2*y3 ;
        y23 = w2*y3 - x2*z3 + y2*w3 + z2*x3 ;
        z23 = w2*z3 + x2*y3 - y2*x3 + z2*w3 ;
        // ROS_INFO("w23= %f , x23= %f , y23= %f , z23= %f \n", w23,x23,y23,z23 );

        // ==> map -> aruco 
        w123 = w12*w3 - x12*x3 - y12*y3 - z12*z3 ;
        x123 = w12*x3 + x12*w3 + y12*z3 - z12*y3 ;
        y123 = w12*y3 - x12*z3 + y12*w3 + z12*x3 ;
        z123 = w12*z3 + x12*y3 - y12*x3 + z12*w3 ;
        // ROS_INFO("w123= %f , x123= %f , y123= %f , z123= %f \n", w123,x123,y123,z123 );

        // goc map - cam
        double roll12, pitch12, yaw12 ;
        tf::Quaternion q12(x12,y12,z12,w12);
        tf::Matrix3x3 m12(q12);
        m12.getRPY(roll12, pitch12, yaw12); // rad
        //  ROS_INFO("roll12= %f , pitch12= %f , yaw12= %f \n", roll12, pitch12, yaw12);

        // goc map->aruco 
        double roll123, pitch123, yaw123 ;
        tf::Quaternion q123(x123,y123,z123,w123);
        tf::Matrix3x3 m123(q123);
        m123.getRPY(roll123, pitch123, yaw123); // rad
        ROS_INFO("map->aruco : roll123= %f , pitch123= %f , yaw123= %f \n", roll123, pitch123, yaw123 );

        // goc cam -> aruco 
        double roll3, pitch3, yaw3 ;
        tf::Quaternion q3(x3,y3,z3,w3);
        tf::Matrix3x3 m3(q3);
        m3.getRPY(roll3, pitch3, yaw3); // rad
        ROS_INFO("cam->aruco : roll3= %f , pitch3= %f , yaw3= %f \n", roll3, pitch3, yaw3);

        // goc robot- aruco 
        double roll23, pitch23, yaw23 ;
        tf::Quaternion q23(x23,y23,z23,w23);
        tf::Matrix3x3 m23(q23);
        m23.getRPY(roll23, pitch23, yaw23); // rad
        ROS_INFO("robot->aruco: roll23= %f , pitch23= %f , yaw23= %f \n", roll23, pitch23, yaw23 );

        // goc robot- map 
        double roll1, pitch1, yaw1 ;
        tf::Quaternion q1(x1,y1,z1,w1);
        tf::Matrix3x3 m1(q1);
        m1.getRPY(roll1, pitch1, yaw1); // rad
        // ROS_INFO("roll1= %f , pitch1= %f , yaw1= %f \n", roll1, pitch1, yaw1 );

        // x' = x*cos(alpha) - y*sin(alpha)
        // y' = x*sin(alpha) + y*cos(alpha)
        // double x = cam_aruco.position.x*cos(yaw123) - cam_aruco.position.z*sin(yaw123) ;
        // double y = cam_aruco.position.x*sin(yaw123) + cam_aruco.position.z*cos(yaw123) ;
        double x = cam_aruco.position.x*cos(yaw123) - cam_aruco.position.z*sin(yaw123) ;
        double y = cam_aruco.position.x*sin(yaw123) + cam_aruco.position.z*cos(yaw123) ;
        // ROS_INFO("x= %f , y= %f \n", x,y );
        // double xx1 = robot_map.position.x + (0.0*cos(yaw12) - 0.265*sin(yaw12)) + x ;
        // double yy1 = robot_map.position.y + (0.0*sin(yaw12) + 0.265*cos(yaw12)) + y ;
        double xx2 = robot_map.position.x + ((0 + cam_aruco.position.x)*cos(yaw12) - (0.265 + cam_aruco.position.z)*sin(yaw12))  ;
        double yy2 = robot_map.position.y + ((0 + cam_aruco.position.x)*sin(yaw12) + (0.265 + cam_aruco.position.z)*cos(yaw12))  ;
        double zz2 = 0.27 + cam_aruco.position.y*cos(roll3); 
        // map -> aruco 
            odom_trans2.header.stamp = current_time;
            odom_trans2.header.frame_id = "map";
            odom_trans2.child_frame_id =  "aru";
            odom_trans2.transform.translation.x = xx2 ;                                          
            odom_trans2.transform.translation.y = yy2 ;
            odom_trans2.transform.translation.z = zz2;
            odom_trans2.transform.rotation.x = x123;
            odom_trans2.transform.rotation.y = y123;
            odom_trans2.transform.rotation.z = z123; 
            odom_trans2.transform.rotation.w = w123; 
            // odom_broadcaster2.sendTransform(odom_trans2);

        
        // map -> my_cam
            odom_trans4.header.stamp = current_time;
            odom_trans4.header.frame_id = "map";
            odom_trans4.child_frame_id =  "my_Cam";
            odom_trans4.transform.translation.x = robot_map.position.x + ( 0.03*cos(yaw12) - 0.265*sin(yaw12) ) ; 
            odom_trans4.transform.translation.y = robot_map.position.y + ( 0.03*sin(yaw12) + 0.265*cos(yaw12) ) ;
            odom_trans4.transform.translation.z = 0.27 ;
            odom_trans4.transform.rotation.x = x12;
            odom_trans4.transform.rotation.y = y12;
            odom_trans4.transform.rotation.z = z12; 
            odom_trans4.transform.rotation.w = w12; 
            // odom_broadcaster4.sendTransform(odom_trans4);

        // cam -> aru 
            odom_trans5.header.stamp = current_time;
            odom_trans5.header.frame_id = "camera_aruco";
            odom_trans5.child_frame_id =  "my_aru";
            odom_trans5.transform.translation.x = 0*cos(yaw3) - 0.092*sin(yaw3) ; 
            odom_trans5.transform.translation.y = 0*sin(yaw3) + 0.092*cos(yaw3) ;
            odom_trans5.transform.translation.z = 0.609;
            odom_trans5.transform.rotation.x = 0.999866517106;
            odom_trans5.transform.rotation.y = -0.00640212496268;
            odom_trans5.transform.rotation.z = -0.00217556013245; 
            odom_trans5.transform.rotation.w = 0.0148737253147; 
            // odom_broadcaster5.sendTransform(odom_trans5);

        // robot-> aru 
            odom_trans6.header.stamp = current_time;
            odom_trans6.header.frame_id = "base_footprint";
            odom_trans6.child_frame_id =  "my_aru1";
            odom_trans6.transform.translation.x =  0 + cam_aruco.position.x ; 
            odom_trans6.transform.translation.y = 0.265 + cam_aruco.position.z;  
            odom_trans6.transform.translation.z = 0.27 + cam_aruco.position.y*cos(roll3);
            odom_trans6.transform.rotation.x = x23 ;
            odom_trans6.transform.rotation.y = y23 ;
            odom_trans6.transform.rotation.z = z23 ; 
            odom_trans6.transform.rotation.w = w23 ;
            odom_broadcaster6.sendTransform(odom_trans6);

        
        r.sleep();
        
    }
}
