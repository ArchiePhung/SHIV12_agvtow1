#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

nav_msgs::Odometry odom_robot,odom_t265 ;

void getOdom_t265(const nav_msgs::Odometry& odom)
{
    odom_t265 = odom ; 
}

int main(int argc, char** argv){
    ros::init(argc, argv, "vector_t265");
    ros::NodeHandle n;
    ros::Publisher pose = n.advertise<nav_msgs::Odometry>("odom_robot_via_t265", 50);
    ros::Subscriber sub = n.subscribe("/t265/odom/sample", 100, getOdom_t265);

    //TF
    tf::TransformBroadcaster        odom_broadcaster1, odom_broadcaster2, odom_broadcaster3;
    geometry_msgs::TransformStamped odom_trans1      , odom_trans2      , odom_trans3;
    
    // robot -> pose
    double x,y,z ;
    tf2::Quaternion qRobot;

    // robot -> T265 
    double xRt = 0.325 ;
    double yRt = 0.0175 ;
    double zRt = 0.34 ;

    // map_ao -> odomT265
        tf2::Quaternion qMo(0,\
                            0,\
                            0,\
                            1);
    // poseT265 -> robot_ao
        tf2::Quaternion qTr(1,\
                            0,\
                            0,\
                            0);

    ros::Time current_time, last_time ;
    
    ros::Rate r(200.0);
    while(n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();
        
        // odomT265 -> poseT265
            tf2::Quaternion qOp(odom_t265.pose.pose.orientation.x,\
                                odom_t265.pose.pose.orientation.y,\
                                odom_t265.pose.pose.orientation.z,\
                                odom_t265.pose.pose.orientation.w ) ;
        

        // map_ao -> robot_ao
            tf2::Quaternion qMr = qMo*qOp*qTr ;

        /// goc map->robot
            double rollMr, pitchMr, yawMr ;
            tf2::Matrix3x3 mMr(qMr);
            mMr.getRPY(rollMr, pitchMr, yawMr); // rad

        double xx = odom_t265.pose.pose.position.x - ( ( xRt)*cos(yawMr) - (-yRt)*sin(yawMr) );
        double yy = odom_t265.pose.pose.position.y - ( ( xRt)*sin(yawMr) + (-yRt)*cos(yawMr) );
        x = xx + xRt;
        y = yy - yRt;
        ROS_INFO("pose->robot: x= %f , y= %f , z= %f \n",x,y,z);

        // map_ao -> rb
            odom_trans1.header.stamp = current_time;
            odom_trans1.header.frame_id = "map_ao";
            odom_trans1.child_frame_id = "rb";      
            odom_trans1.transform.translation.x = xx;                                          
            odom_trans1.transform.translation.y = yy;
            odom_trans1.transform.translation.z = 0;
            odom_trans1.transform.rotation.x = qMr[0] ;
            odom_trans1.transform.rotation.y = qMr[1];
            odom_trans1.transform.rotation.z = qMr[2]; 
            odom_trans1.transform.rotation.w = qMr[3];
            odom_broadcaster1.sendTransform(odom_trans1);

        // pose -> map_ao 
            odom_trans2.header.stamp = current_time;
            odom_trans2.header.frame_id = "t265_pose_frame";
            odom_trans2.child_frame_id = "robot_ao";      
            odom_trans2.transform.translation.x = -xRt;                                          
            odom_trans2.transform.translation.y = -yRt;
            odom_trans2.transform.translation.z = 0;
            odom_trans2.transform.rotation.x = qTr[0] ;
            odom_trans2.transform.rotation.y = qTr[1];
            odom_trans2.transform.rotation.z = qTr[2]; 
            odom_trans2.transform.rotation.w = qTr[3];
            // odom_broadcaster2.sendTransform(odom_trans2);

        // map_ao -> rb_that
            odom_trans3.header.stamp = current_time;
            odom_trans3.header.frame_id = "map_ao";
            odom_trans3.child_frame_id = "rb_that";      
            odom_trans3.transform.translation.x = x ;                                          
            odom_trans3.transform.translation.y = y ;
            odom_trans3.transform.rotation.x = qMr[0] ;
            odom_trans3.transform.rotation.y = qMr[1];
            odom_trans3.transform.rotation.z = qMr[2]; 
            odom_trans3.transform.rotation.w = qMr[3];
            // odom_broadcaster3.sendTransform(odom_trans3);

        // publish
            odom_robot.header.stamp = current_time;
            odom_robot.header.frame_id = "odom";
            odom_robot.child_frame_id = "base_footprint";
            odom_robot.pose.pose.position.x = x;
            odom_robot.pose.pose.position.y = y;
            odom_robot.pose.pose.position.z = 0.0;
            odom_robot.pose.pose.orientation.x = qMr[0];
            odom_robot.pose.pose.orientation.y = qMr[1];
            odom_robot.pose.pose.orientation.z = qMr[2]; 
            odom_robot.pose.pose.orientation.w = qMr[3];
            odom_robot.pose.covariance = odom_t265.pose.covariance ;
            
            pose.publish(odom_robot);

       
       
        r.sleep();
        
    }
}
