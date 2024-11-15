
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
bool enable_tf = false , calculator = false;
bool sub4 = false ;

geometry_msgs::Pose aruco1_map,aruco2_map,aruco4_map,aruco5_map; 
geometry_msgs::PoseWithCovarianceStamped pose_rb ;

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

    ros::Time current_time, last_time ;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(10.0);
    double secs1 =ros::Time::now().toSec();
    double secs2 = ros::Time::now().toSec() ;

    // aruco1
        aruco1_map.position.x = -1.277973 ;
        aruco1_map.position.y = 1.619685  ;
        aruco1_map.position.z = 0.218695   ;
        tf2::Quaternion qMapAr1( 0.687331 ,0.005052 ,-0.003179 ,0.726320);

    // aruco2
        aruco2_map.position.x = -2.096373 ;
        aruco2_map.position.y = 2.056464  ;
        aruco2_map.position.z = 0.305218 ;
        tf2::Quaternion qMapAr2( 0.697445 ,0.006396 ,-0.039896 ,0.715498);
    // aruco4
        aruco4_map.position.x = 2.513777  ;
        aruco4_map.position.y = -1.251224 ;
        aruco4_map.position.z = 0.236931  ;
        tf2::Quaternion qMapAr4(0.503722 ,-0.499309 ,-0.504260 ,0.492622);

    // aruco5
        aruco5_map.position.x = 3.180299  ;
        aruco5_map.position.y = -0.426177 ;
        aruco5_map.position.z = 0.282627 ;
        tf2::Quaternion qMapAr5(0.519341 ,-0.502970 ,-0.486331 ,0.490701);
  


    while(n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();
        int id_aruco ;
       //double a,b,c,d ; 
        double xx_rb , yy_rb ;
        double x_rb ,y_rb ;
        // quata map ->cam
        tf2::Quaternion qMapCam , qAruRobot, qMapAru;
        // map -> robot ( can tim )
        tf2::Quaternion qrb;

        if (sub4 == true){   
            id_aruco = getPosearuco.detections[0].id[0] ;
            aruco = getPosearuco.detections[0].pose.pose.pose;
            // ROS_INFO("id = %d \n",id_aruco );
            sub4 = false ;

            tf2::Quaternion qCamAr(aruco.orientation.x,\
                                   aruco.orientation.y,\
                                   aruco.orientation.z,\
                                   aruco.orientation.w );
            double rollCa, pitchCa, yawCa ;
            tf2::Matrix3x3 mCa(qCamAr);
            mCa.getRPY(rollCa, pitchCa, yawCa); // rad
            // ROS_INFO("pitchCa = %f \n",pitchCa );

            //check
            int step = 0 ;
            if (step == 0){
                if ( id_aruco ==1 ||id_aruco ==2 || id_aruco ==4 || id_aruco ==5 ) step = 1 ;
            }
            if (step == 1){
                if (pitchCa < 0.2 && pitchCa > -0.2 ) step = 2 ;
            }
            if (step == 2){
                if (aruco.position.x < 0.3 && aruco.position.x > -0.3 ) step = 10 ;
            }
            
            if (step == 10 ) {
                enable_tf = true ;
                step =0 ;
            }
            else enable_tf = false ;
            // ROS_INFO("enable_tf = %d \n",enable_tf );
        }
        if (enable_tf == true ){

            
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

            

            if (id_aruco == 1 ) {
                qMapCam = qMapAr1 * qArCam ;
                qrb     = qMapCam * q2;
                qMapAru = qMapAr1 ;
            }
            if (id_aruco == 2 ) {
                qMapCam = qMapAr2 * qArCam ;
                qrb     = qMapCam * q2;
                qMapAru = qMapAr2 ;
            }
            if (id_aruco == 4 ) {
                qMapCam = qMapAr4 * qArCam ;
                qrb     = qMapCam * q2;
                qMapAru = qMapAr4 ;
            }
            if (id_aruco == 5 ) {
                qMapCam = qMapAr5 * qArCam ;
                qrb     = qMapCam * q2;
                qMapAru = qMapAr5 ;
                
            }


         /// goc map<-robot
            double rollRm, pitchRm, yawRm ;
            tf2::Matrix3x3 mRm(qrb);
            mRm.getRPY(rollRm, pitchRm, yawRm); // rad
        

         /// goc map <- cam
            tf2::Quaternion qCammap(-1* qMapCam[0], -1* qMapCam[1], -1* qMapCam[2], qMapCam[3]);
            double rollCm, pitchCm, yawCm ;
            tf2::Matrix3x3 mCm(qCammap);
            mCm.getRPY(rollCm, pitchCm, yawCm ); // rad


            
            if (id_aruco == 1 ) {
                xx_rb = aruco1_map.position.x -  aruco.position.x*cos(pitchCm) - (-1* aruco.position.z*sin(pitchCm)) ;
                yy_rb = aruco1_map.position.y -  aruco.position.x*sin(pitchCm) + (-1* aruco.position.z*cos(pitchCm)) ;          
                odom_trans1.child_frame_id = "ar1";
                odom_trans1.transform.translation.x = aruco1_map.position.x;                                          
                odom_trans1.transform.translation.y = aruco1_map.position.y;
                odom_trans1.transform.translation.z = aruco1_map.position.z;
            }
            if (id_aruco == 2 ) {
                xx_rb = aruco2_map.position.x -  aruco.position.x*cos(pitchCm) - (-1* aruco.position.z*sin(pitchCm)) ;
                yy_rb = aruco2_map.position.y -  aruco.position.x*sin(pitchCm) + (-1* aruco.position.z*cos(pitchCm)) ; 
                odom_trans1.child_frame_id = "ar2";
                odom_trans1.transform.translation.x = aruco2_map.position.x;                                          
                odom_trans1.transform.translation.y = aruco2_map.position.y;
                odom_trans1.transform.translation.z = aruco2_map.position.z;
            }
            if (id_aruco == 4 ) {
                xx_rb = aruco4_map.position.x -  aruco.position.x*cos(pitchCm) - (-1* aruco.position.z*sin(pitchCm)) ;
                yy_rb = aruco4_map.position.y -  aruco.position.x*sin(pitchCm) + (-1* aruco.position.z*cos(pitchCm)) ; 
                odom_trans1.child_frame_id = "ar4";
                odom_trans1.transform.translation.x = aruco4_map.position.x;                                          
                odom_trans1.transform.translation.y = aruco4_map.position.y;
                odom_trans1.transform.translation.z = aruco4_map.position.z;
            }
            if (id_aruco == 5 ) {
                xx_rb = aruco5_map.position.x -  aruco.position.x*cos(pitchCm) - (-1* aruco.position.z*sin(pitchCm)) ;
                yy_rb = aruco5_map.position.y -  aruco.position.x*sin(pitchCm) + (-1* aruco.position.z*cos(pitchCm)) ; 
                odom_trans1.child_frame_id = "ar5";      
                odom_trans1.transform.translation.x = aruco5_map.position.x;                                          
                odom_trans1.transform.translation.y = aruco5_map.position.y;
                odom_trans1.transform.translation.z = aruco5_map.position.z;
            }

            x_rb = xx_rb + 0*cos(yawRm) - (-1* 0.265*sin(yawRm));
            y_rb = yy_rb + 0*sin(yawRm) + (-1* 0.265*cos(yawRm)) ;
            

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

            
            secs1 =ros::Time::now().toSec();
            if ( secs1 - secs2 > 0.5) {
                pose_rb.pose.pose.position.x = x_rb ; 
                pose_rb.pose.pose.position.y = y_rb ; 
                pose_rb.pose.pose.orientation.x = qrb[0]; 
                pose_rb.pose.pose.orientation.y = qrb[1];
                pose_rb.pose.pose.orientation.z = qrb[2];
                pose_rb.pose.pose.orientation.w = qrb[3];
                setpost.publish(pose_rb);
                ROS_INFO("--> set poseid= %d , x_rb= %f , y_rb= %f",id_aruco,x_rb ,y_rb );

                secs2 = ros::Time::now().toSec() ; 
                // ROS_INFO("secs1 = %f, secs2 = %f",secs1,secs2);
            }
        }
        

        r.sleep();
        
    }
}
