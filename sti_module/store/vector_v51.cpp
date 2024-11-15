
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
geometry_msgs::Pose aruco ; 

geometry_msgs::Pose robot_map; 
float tx, ty, tz, ox, oy, oz, ow ;
geometry_msgs::TransformStamped trans1 ;
bool enable_tf = false ;
bool sub4 = false ;

geometry_msgs::Pose aruco1_map,aruco21_map,aruco22_map,aruco4_map; 


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
    ros::Publisher setpost = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 50);
    
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

    // // aruco1
        //     aruco1_map.position.x = -1.166526 ;
        //     aruco1_map.position.y = 1.075501 ;
        //     aruco1_map.position.z = 0.172078 ;
        //     tf2::Quaternion qMapAr1(0.680259 ,0.006534 ,0.011023 ,0.732860 );

        // // aruco21
        //     aruco21_map.position.x = -1.839964 ;
        //     aruco21_map.position.y = 1.481630 ;
        //     aruco21_map.position.z = 0.300891 ;
        //     tf2::Quaternion qMapAr21(0.711143 ,0.009380 ,-0.009023 ,0.702926  );
        // // aruco22
        //     aruco22_map.position.x = 3.835525 ;
        //     aruco22_map.position.y = 0.108077 ;
        //     aruco22_map.position.z = 0.538582 ;
        //     tf2::Quaternion qMapAr22(0.509918 ,-0.490353 ,-0.492860 ,0.506583  );
        // // aruco4
        //     aruco4_map.position.x = 2.890752 ;
        //     aruco4_map.position.y = -0.560028 ;
        //     aruco4_map.position.z = 0.179440  ;
        //     tf2::Quaternion qMapAr4(0.556619 ,-0.488329 ,-0.454749 ,0.494887);

    // aruco1
        aruco1_map.position.x = -2.080842  ;
        aruco1_map.position.y = 1.129233;
        aruco1_map.position.z = 0.175051 ;
        tf2::Quaternion qMapAr1(0.658255 ,0.192566 ,0.200828 ,0.699490 );

    // aruco21
        aruco21_map.position.x = -2.875791 ;
        aruco21_map.position.y = 1.193658  ;
        aruco21_map.position.z = 0.301144 ;
        tf2::Quaternion qMapAr21(0.698579 ,0.201725 , 0.176643 ,0.663394   );
    // aruco22
        aruco22_map.position.x = 2.916292 ;
        aruco22_map.position.y = 2.505384 ;
        aruco22_map.position.z = 0.538214 ;
        tf2::Quaternion qMapAr22(0.618471 ,-0.371282 ,-0.353013 ,0.595840 
  );
    // aruco4
        aruco4_map.position.x = 2.349072  ;
        aruco4_map.position.y = 1.489660 ;
        aruco4_map.position.z = 0.178495  ;
        tf2::Quaternion qMapAr4(0.611097 ,-0.339124 ,-0.367096 ,0.613837 );

    while(n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();
        int id_aruco ;
        double a,b,c,d ; 
        if (sub4 == true){   
            id_aruco = getPosearuco.detections[0].id[0] ;
            aruco = getPosearuco.detections[0].pose.pose.pose;
            // ROS_INFO("id = %d \n",id_aruco );
            sub4 = false ;

            // a = getPosearuco.detections[0].id[0] ;
            // b = getPosearuco.detections[1].id[0] ;
            // c = getPosearuco.detections[2].id[0] ;
            // d = getPosearuco.detections[3].id[0] ;
            // ROS_INFO("a= %f, b= %f, c= %f, d= %f, \n",a,b,c,d );
        }

        // cam -> robot 
        tf2::Quaternion q2((1 * can2_2 ),\
                           0,\
                           0,\
                           can2_2);
        //robot - > cam 
        tf2::Quaternion q2_n((-1 * can2_2 ),\
                           0,\
                           0,\
                           can2_2);
        // aruco1 -> cam  
        tf2::Quaternion qArCam(-aruco.orientation.x,\
                               -aruco.orientation.y,\
                               -aruco.orientation.z,\
                               aruco.orientation.w );

        // quata map ->cam
        tf2::Quaternion qMapCam , qAruRobot, qMapAru;
        // map -> robot ( can tim )
        tf2::Quaternion qrb;

        if (id_aruco == 1 ) {
            qMapCam = qMapAr1 * qArCam ;
            qrb     = qMapCam * q2;
            qMapAru = qMapAr1 ;
        }
        if (id_aruco == 21 ) {
            qMapCam = qMapAr21 * qArCam ;
            qrb     = qMapCam * q2;
            qMapAru = qMapAr21 ;
        }
        if (id_aruco == 22 ) {
            qMapCam = qMapAr22 * qArCam ;
            qrb     = qMapCam * q2;
            qMapAru = qMapAr22 ;
        }
        if (id_aruco == 4 ) {
            qMapCam = qMapAr4 * qArCam ;
            qrb     = qMapCam * q2;
            qMapAru = qMapAr4 ;
            
        }

        // goc Aru -> Robot
        qAruRobot = qArCam * q2;
        // double rollMc, pitchMc, yawArMc ;
        // tf2::Matrix3x3 mMc(qAruRobot);
        // mMc.getRPY(rollMc, pitchMc, yawArMc); // rad

        // goc Aru <- Robot
        tf2::Quaternion qRobotAru(-1* qAruRobot[0], -1* qAruRobot[1], -1* qAruRobot[2], qAruRobot[3]) ;
        double rollRa, pitchRa, yawArRa ;
        tf2::Matrix3x3 mRa(qRobotAru);
        mRa.getRPY(rollRa, pitchRa, yawArRa); // rad

        // goc Aru->cam
        double rollAc, pitchAc, yawAc ;
        tf2::Matrix3x3 mAc(qArCam);
        mAc.getRPY(rollAc, pitchAc, yawAc); // rad

        // goc map->robot
        double rollMr, pitchMr, yawMr ;
        tf2::Quaternion qrb_n(-1* qrb[0], -1* qrb[1], -1* qrb[2],qrb[3]) ;
        tf2::Matrix3x3 mMr(qrb_n);
        mMr.getRPY(rollMr, pitchMr, yawMr); // rad
        // ROS_INFO("map->robot :id= %d , rollMr= %f , pitchMr= %f , yawMr= %f ",id_aruco,rollMr ,pitchMr,yawMr  );

        // goc map<-robot
        double rollRm, pitchRm, yawRm ;
        tf2::Matrix3x3 mRm(qrb);
        mRm.getRPY(rollRm, pitchRm, yawRm); // rad
       

        // goc map -> Aru
        double rollMa, pitchMa, yawMa ;
        tf2::Matrix3x3 mMa(qMapAru);
        mMa.getRPY(rollMa, pitchMa, yawMa); // rad

        // goc map <-Aru
        tf2::Quaternion qMapAru_n(-1* qMapAr1[0], -1* qMapAr1[1], -1* qMapAr1[2],qMapAr1[3]) ;
        double rollAm, pitchAm, yawAm ;
        tf2::Matrix3x3 mAm(qMapAru_n);
        mAm.getRPY(rollAm, pitchAm, yawAm ); // rad

        // goc map -> cam
        double rollMc, pitchMc, yawMc ;
        tf2::Matrix3x3 mMc(qMapCam);
        mMc.getRPY(rollMc, pitchMc, yawMc); // rad

        // goc map <- cam
        tf2::Quaternion qCammap(-1* qMapCam[0], -1* qMapCam[1], -1* qMapCam[2], qMapCam[3]);
        double rollCm, pitchCm, yawCm ;
        tf2::Matrix3x3 mCm(qCammap);
        mCm.getRPY(rollCm, pitchCm, yawCm ); // rad


        double x =  0*cos(pitchAc) - 0.265*sin(pitchAc) ;
        double y =  0*sin(pitchAc) + 0.265*cos(pitchAc) ;
        double x1 =  aruco.position.x*cos(pitchAc) - aruco.position.z*sin(pitchAc) ;
        double y1 =  aruco.position.x*sin(pitchAc) + aruco.position.z*cos(pitchAc) ;
        double xx =  x1 + x ;
        double yy =  y1 + y;

        ROS_INFO("xx= %f , yy= %f",xx ,yy );

        // double x_Ar1 = robot_map.position.x + ((0 + aruco.position.x)*cos(yawArMc) - (0.265 + aruco.position.z)*sin(yawArMc))  ;
        // double y_Ar1 = robot_map.position.y + ((0 + aruco.position.x)*sin(yawArMc) + (0.265 + aruco.position.z)*cos(yawArMc))  ;
        double xx_rb , yy_rb ;
        double x_rb ,y_rb ;
        if (id_aruco == 1 ) {
            xx_rb = aruco1_map.position.x -  aruco.position.x*cos(pitchCm) - (-1* aruco.position.z*sin(pitchCm)) ;
            yy_rb = aruco1_map.position.y -  aruco.position.x*sin(pitchCm) + (-1* aruco.position.z*cos(pitchCm)) ;          
            odom_trans1.child_frame_id = "ar1";
            odom_trans1.transform.translation.x = aruco1_map.position.x;                                          
            odom_trans1.transform.translation.y = aruco1_map.position.y;
            odom_trans1.transform.translation.z = aruco1_map.position.z;
        }
        if (id_aruco == 21 ) {
            xx_rb = aruco21_map.position.x -  aruco.position.x*cos(pitchCm) - (-1* aruco.position.z*sin(pitchCm)) ;
            yy_rb = aruco21_map.position.y -  aruco.position.x*sin(pitchCm) + (-1* aruco.position.z*cos(pitchCm)) ; 
            odom_trans1.child_frame_id = "ar21";
            odom_trans1.transform.translation.x = aruco21_map.position.x;                                          
            odom_trans1.transform.translation.y = aruco21_map.position.y;
            odom_trans1.transform.translation.z = aruco21_map.position.z;
        }
        if (id_aruco == 22 ) {
            xx_rb = aruco22_map.position.x -  aruco.position.x*cos(pitchCm) - (-1* aruco.position.z*sin(pitchCm)) ;
            yy_rb = aruco22_map.position.y -  aruco.position.x*sin(pitchCm) + (-1* aruco.position.z*cos(pitchCm)) ; 
            odom_trans1.child_frame_id = "ar22";
            odom_trans1.transform.translation.x = aruco22_map.position.x;                                          
            odom_trans1.transform.translation.y = aruco22_map.position.y;
            odom_trans1.transform.translation.z = aruco22_map.position.z;
        }
        if (id_aruco == 4 ) {
            xx_rb = aruco4_map.position.x -  aruco.position.x*cos(pitchCm) - (-1* aruco.position.z*sin(pitchCm)) ;
            yy_rb = aruco4_map.position.y -  aruco.position.x*sin(pitchCm) + (-1* aruco.position.z*cos(pitchCm)) ; 
            odom_trans1.child_frame_id = "ar4";      
            odom_trans1.transform.translation.x = aruco4_map.position.x;                                          
            odom_trans1.transform.translation.y = aruco4_map.position.y;
            odom_trans1.transform.translation.z = aruco4_map.position.z;
        }

        x_rb = xx_rb + 0*cos(yawRm) - (-1* 0.265*sin(yawRm));
        y_rb = yy_rb + 0*sin(yawRm) + (-1* 0.265*cos(yawRm)) ;
        ROS_INFO("id= %d , x_rb= %f , y_rb= %f",id_aruco,x_rb ,y_rb );
        
        // ROS_INFO("cam->robot :id= %d , x= %f , y= %f , z= %f ",id_aruco,xx ,yy,z_rb  );
        
        // aru ->  map 
            odom_trans3.header.stamp = current_time;
            odom_trans3.header.frame_id = "tag_1";
            odom_trans3.child_frame_id =  "map1";
            odom_trans3.transform.translation.x = -(aruco1_map.position.x*cos(pitchAm) - aruco1_map.position.y*sin(pitchAm) );                                          
            odom_trans3.transform.translation.y =   aruco1_map.position.x*sin(pitchAm) + aruco1_map.position.y*cos(pitchAm) ;
            odom_trans3.transform.translation.z = y_rb ;
            odom_trans3.transform.rotation.x = qMapAru_n[0] ;
            odom_trans3.transform.rotation.y = qMapAru_n[1] ;
            odom_trans3.transform.rotation.z = qMapAru_n[2] ; 
            odom_trans3.transform.rotation.w = qMapAru_n[3] ;
            // odom_broadcaster3.sendTransform(odom_trans3);


        // aru_1 -> cam
            odom_trans2.header.stamp = current_time;
            odom_trans2.header.frame_id = "tag_1";
            odom_trans2.child_frame_id =  "robot";
            odom_trans2.transform.translation.x = -xx ;                                          
            odom_trans2.transform.translation.y = 0;
            odom_trans2.transform.translation.z = yy ;
            odom_trans2.transform.rotation.x = qAruRobot[0] ;
            odom_trans2.transform.rotation.y = qAruRobot[1];
            odom_trans2.transform.rotation.z = qAruRobot[2]; 
            odom_trans2.transform.rotation.w = qAruRobot[3];
            // odom_broadcaster2.sendTransform(odom_trans2);

        // map -> aru_1 
            odom_trans1.header.stamp = current_time;
            odom_trans1.header.frame_id = "map";
            // odom_trans1.child_frame_id = "ar4";      
            // odom_trans1.transform.translation.x = aruco4_map.position.x;                                          
            // odom_trans1.transform.translation.y = aruco4_map.position.y;
            // odom_trans1.transform.translation.z = aruco4_map.position.z;
            odom_trans1.transform.rotation.x = qMapAru[0] ;
            odom_trans1.transform.rotation.y = qMapAru[1];
            odom_trans1.transform.rotation.z = qMapAru[2]; 
            odom_trans1.transform.rotation.w = qMapAru[3];
            odom_broadcaster1.sendTransform(odom_trans1);

        // aru_1 -> cam
            odom_trans4.header.stamp = current_time;
            odom_trans4.header.frame_id = "map";
            odom_trans4.child_frame_id =  "cam";
            odom_trans4.transform.translation.x = xx_rb;                                          
            odom_trans4.transform.translation.y = yy_rb ;
            odom_trans4.transform.translation.z = 0 ;
            odom_trans4.transform.rotation.x = qMapCam[0] ;
            odom_trans4.transform.rotation.y = qMapCam[1];
            odom_trans4.transform.rotation.z = qMapCam[2]; 
            odom_trans4.transform.rotation.w = qMapCam[3];
            // odom_broadcaster4.sendTransform(odom_trans4);

        // aru_1 -> cam
            odom_trans5.header.stamp = current_time;
            odom_trans5.header.frame_id = "map";
            odom_trans5.child_frame_id =  "robot";
            odom_trans5.transform.translation.x = x_rb;                                          
            odom_trans5.transform.translation.y = y_rb ;
            odom_trans5.transform.translation.z = 0 ;
            odom_trans5.transform.rotation.x = qrb[0] ;
            odom_trans5.transform.rotation.y = qrb[1];
            odom_trans5.transform.rotation.z = qrb[2]; 
            odom_trans5.transform.rotation.w = qrb[3];
            odom_broadcaster5.sendTransform(odom_trans5);


        r.sleep();
        
    }
}
