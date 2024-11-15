#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace std;
fstream f;
geometry_msgs::PoseStamped robotPose;
geometry_msgs::PoseWithCovarianceStamped poseRb; 

#define pi 3.141592653589793238462643383279502884
#define can2_2 0.7071067812
apriltag_ros::AprilTagDetectionArray getPosearuco; 
string file = "/home/stivietnam/catkin_ws/src/vector_convert/data/data_tag.txt" ;
bool sub4 = false ,sub5 = false ;

// call
    void callSub4(const apriltag_ros::AprilTagDetectionArray& pose){ 
        getPosearuco = pose ; 
        sub4 = true ;
    }

// read file
    int findData(geometry_msgs::PoseStamped& tag_data, string tag_name){ 
        f.open(file, ios::in | ios::out | ios::app);
        if (f.is_open()) // check file is open
        {
            string data1,data2,data3,data4,data5,data6,data7;
            string line;
            int dem = 0 ; 
            // f.seekg(ios_base::beg);
            while (!f.eof()) // kiem tra da ket thuc file chua
            {
                getline(f, line);
                if(line == tag_name){
                    // get data in line
                    getline(f, data1);
                    getline(f, data2);
                    getline(f, data3);
                    getline(f, data4);
                    getline(f, data5);
                    getline(f, data6);
                    getline(f, data7);

                    // cut data
                    data1 = data1.substr(data1.find("=") + 2,10);     
                    data2 = data2.substr(data2.find("=") + 2,10);
                    data3 = data3.substr(data3.find("=") + 2,10);              
                    data4 = data4.substr(data4.find("=") + 2,10);
                    data5 = data5.substr(data5.find("=") + 2,10);
                    data6 = data6.substr(data6.find("=") + 2,10);
                    data7 = data7.substr(data7.find("=") + 2,10);

                    // convert string -> float
                    *(&tag_data.header.frame_id)    =   tag_name ;
                    *(&tag_data.pose.position.x)    =   stof(data1.c_str());
                    *(&tag_data.pose.position.y)    =   stof(data2.c_str());
                    *(&tag_data.pose.position.z)    =   stof(data3.c_str());
                    *(&tag_data.pose.orientation.x) =   stof(data4.c_str());
                    *(&tag_data.pose.orientation.y) =   stof(data5.c_str());
                    *(&tag_data.pose.orientation.z) =   stof(data6.c_str());
                    *(&tag_data.pose.orientation.w) =   stof(data7.c_str());

                    // ROS_INFO("this is");
                    dem ++ ;
                    f.close();
           
                }
            }
            if(f.eof()){
                return dem ;
                
            }
        }
        else
        {
            return -1; //  Error opening file 
        }
  
    }
// STI tf
    void stiTf_Data(geometry_msgs::PoseStamped& tag_data, string origin_frame, string target_frame)
    {   
        tf::TransformBroadcaster        broadcaster;
        geometry_msgs::TransformStamped trans      ;
        tf::TransformListener           listener   ;
        tf::StampedTransform            transform  ;
        string origin_frame_f = origin_frame + "_f";
        string target_frame_f = target_frame + "_f";

        listener.waitForTransform(origin_frame, target_frame_f, ros::Time(), ros::Duration(1));
        try{
            listener.lookupTransform(origin_frame, target_frame_f,ros::Time(0), transform);

            trans.header.stamp            = ros::Time::now();
            trans.header.frame_id         = origin_frame;
            trans.child_frame_id          = target_frame_f;
            trans.transform.translation.x = transform.getOrigin().x() ;                                            
            trans.transform.translation.y = transform.getOrigin().y() ;
            trans.transform.translation.z = transform.getOrigin().z() ;
            trans.transform.rotation.x    = transform.getRotation().x() ;
            trans.transform.rotation.y    = transform.getRotation().y() ;
            trans.transform.rotation.z    = transform.getRotation().z() ; 
            trans.transform.rotation.w    = transform.getRotation().w() ;                
            broadcaster.sendTransform(trans);

            *(&tag_data.header.frame_id)    = target_frame;
            *(&tag_data.pose.position.x)    = transform.getOrigin().x() ;    
            *(&tag_data.pose.position.y)    = transform.getOrigin().y() ;
            *(&tag_data.pose.position.z)    = transform.getOrigin().z() ;
            *(&tag_data.pose.orientation.x) = transform.getRotation().x() ;
            *(&tag_data.pose.orientation.y) = transform.getRotation().y() ;
            *(&tag_data.pose.orientation.z) = transform.getRotation().z() ; 
            *(&tag_data.pose.orientation.w) = transform.getRotation().w() ;   
            //  return tag_data ;   
                    
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
    }

    void stiTf_Frame(string origin_frame, string target1_frame, string target2_frame)
    {   
        tf::TransformBroadcaster        broadcaster , broadcaster1 ,broadcaster2 ;
        geometry_msgs::TransformStamped trans       , trans1       ,trans2       ;
        tf::TransformListener           listener    , listener1    ,listener2    ;
        tf::StampedTransform            transform   , transform1   ,transform2   ;
        string origin_frame_f = origin_frame + "_f";
        string target1_frame_f = target1_frame + "_f_" + origin_frame;
        string target2_frame_f = target2_frame + "_f_" + origin_frame;

        // tag -> cam
        listener.waitForTransform(origin_frame, target1_frame, ros::Time(), ros::Duration(1));
        try{
            listener.lookupTransform(origin_frame, target1_frame,ros::Time(0), transform);

            trans.header.stamp            = ros::Time::now();
            trans.header.frame_id         = origin_frame_f;
            trans.child_frame_id          = target1_frame_f;
            trans.transform.translation.x = transform.getOrigin().x() ;                                            
            trans.transform.translation.y = transform.getOrigin().y() ;
            trans.transform.translation.z = transform.getOrigin().z() ;
            trans.transform.rotation.x    = transform.getRotation().x() ;
            trans.transform.rotation.y    = transform.getRotation().y() ;
            trans.transform.rotation.z    = transform.getRotation().z() ; 
            trans.transform.rotation.w    = transform.getRotation().w() ;                
            broadcaster.sendTransform(trans);
                    
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        // cam - base_footprint
        listener1.waitForTransform(target1_frame, target2_frame, ros::Time(), ros::Duration(1));
        try{
            listener1.lookupTransform(target1_frame, target2_frame,ros::Time(0), transform1);

            trans1.header.stamp            = ros::Time::now();
            trans1.header.frame_id         = target1_frame_f;
            trans1.child_frame_id          = target2_frame_f;
            trans1.transform.translation.x = transform1.getOrigin().x() ;                                            
            trans1.transform.translation.y = transform1.getOrigin().y() ;
            trans1.transform.translation.z = transform1.getOrigin().z() ;
            trans1.transform.rotation.x    = transform1.getRotation().x() ;
            trans1.transform.rotation.y    = transform1.getRotation().y() ;
            trans1.transform.rotation.z    = transform1.getRotation().z() ; 
            trans1.transform.rotation.w    = transform1.getRotation().w() ;                
            broadcaster1.sendTransform(trans1);
                    
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        // map - base_footprint
        listener2.waitForTransform("/map", target2_frame_f, ros::Time(), ros::Duration(1));
        try{
            listener2.lookupTransform("/map", target2_frame_f,ros::Time(0), transform2);

            trans2.header.stamp            = ros::Time::now();
            trans2.header.frame_id         = "/map";
            trans2.child_frame_id          = target2_frame_f;
            trans2.transform.translation.x = transform2.getOrigin().x() ;                                            
            trans2.transform.translation.y = transform2.getOrigin().y() ;
            trans2.transform.translation.z = transform2.getOrigin().z() ;
            trans2.transform.rotation.x    = transform2.getRotation().x() ;
            trans2.transform.rotation.y    = transform2.getRotation().y() ;
            trans2.transform.rotation.z    = transform2.getRotation().z() ; 
            trans2.transform.rotation.w    = transform2.getRotation().w() ;                
            // broadcaster2.sendTransform(trans2);
            
            // data
            // *(&tag_data.header.frame_id)    = target_frame;
            // *(&tag_data.pose.position.x)    = transform.getOrigin().x() ;    
            // *(&tag_data.pose.position.y)    = transform.getOrigin().y() ;
            // *(&tag_data.pose.position.z)    = transform.getOrigin().z() ;
            // *(&tag_data.pose.orientation.x) = transform.getRotation().x() ;
            // *(&tag_data.pose.orientation.y) = transform.getRotation().y() ;
            // *(&tag_data.pose.orientation.z) = transform.getRotation().z() ; 
            // *(&tag_data.pose.orientation.w) = transform.getRotation().w() ;      
                    
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
    }

    void stiBroadcaster(geometry_msgs::PoseStamped tag_data, string origin_frame, string target_frame)
    {
        tf::TransformBroadcaster        broadcaster;
        geometry_msgs::TransformStamped trans      ;
        string origin_frame_f = origin_frame + "_f";
        string target_frame_f = target_frame + "_f";

        trans.header.stamp = ros::Time::now();
        trans.header.frame_id = origin_frame;
        trans.child_frame_id =  target_frame_f;
        trans.transform.translation.x = tag_data.pose.position.x  ;                                         
        trans.transform.translation.y = tag_data.pose.position.y  ;
        trans.transform.translation.z = tag_data.pose.position.z  ;
        trans.transform.rotation.x    = tag_data.pose.orientation.x ;
        trans.transform.rotation.y    = tag_data.pose.orientation.y ;
        trans.transform.rotation.z    = tag_data.pose.orientation.z ;
        trans.transform.rotation.w    = tag_data.pose.orientation.w ;
        broadcaster.sendTransform(trans);
    }

    void loadDtata(vector<geometry_msgs::PoseStamped>& pose, vector<string> tag){
        int sl_tag = tag.size();
        printf(" sl_tag %d",sl_tag);

        if (f.is_open()) // check file is open
        {   
            string data1,data2,data3,data4,data5,data6,data7;
            string line;
            int dem = 0 ; 
            while (!f.eof()) // kiem tra da ket thuc file chua
            {               
                
                getline(f, line);
                for( int i = 0 ; i < sl_tag ; i++){
                    if(line == tag.at(i)){
                        
                        printf(" found %s",tag.at(i).c_str());
                        // get data in line
                        getline(f, data1);
                        getline(f, data2);
                        getline(f, data3);
                        getline(f, data4);
                        getline(f, data5);
                        getline(f, data6);
                        getline(f, data7);

                        // cut data
                        data1 = data1.substr(data1.find("=") + 2,10);     
                        data2 = data2.substr(data2.find("=") + 2,10);
                        data3 = data3.substr(data3.find("=") + 2,10);              
                        data4 = data4.substr(data4.find("=") + 2,10);
                        data5 = data5.substr(data5.find("=") + 2,10);
                        data6 = data6.substr(data6.find("=") + 2,10);
                        data7 = data7.substr(data7.find("=") + 2,10);

                        // convert string -> float
                        *(&pose.at(i).header.frame_id)    =   tag.at(i) ;
                        *(&pose.at(i).pose.position.x)    =   stof(data1.c_str());
                        *(&pose.at(i).pose.position.y)    =   stof(data2.c_str());
                        *(&pose.at(i).pose.position.z)    =   stof(data3.c_str());
                        *(&pose.at(i).pose.orientation.x) =   stof(data4.c_str());
                        *(&pose.at(i).pose.orientation.y) =   stof(data5.c_str());
                        *(&pose.at(i).pose.orientation.z) =   stof(data6.c_str());
                        *(&pose.at(i).pose.orientation.w) =   stof(data7.c_str());

                    }
                }
            }
        }
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
    // file 
        // f.open(file, ios::in | ios::out | ios::app);
    // string
        string map_frame,tag_name,tag_fistName,cam_name, robot_frame;
        map_frame    = "/map";
        tag_fistName = "/tag_";
        cam_name     = "/camera_aruco";
        robot_frame  = "/base_footprint";
    // find data
        vector<string> listTag (4);
        listTag.at(0) = "/tag_10" ;
        listTag.at(1) = "/tag_11" ;
        listTag.at(2) = "/tag_3" ;
        listTag.at(3) = "/tag_4" ;
        vector<geometry_msgs::PoseStamped> tagpose (4);

        printf("\n\n               Load file .... \n\n");
        f.open(file, ios::in );
        loadDtata(tagpose,listTag);
        printf("\n\n                Load file Ok :)\n\n ");
        f.close();
        // Map for speed keys
        map<string, geometry_msgs::PoseStamped> mapTagPose 
        {
            {"/tag_10", tagpose.at(0)},
            {"/tag_11", tagpose.at(1)},
            {"/tag_3", tagpose.at(2)},
            {"/tag_4", tagpose.at(3)},
        };
        tf::TransformBroadcaster        broadcaster;
        geometry_msgs::TransformStamped trans      ;

    while(n.ok()){
        ros::spinOnce();
        // ros::spin();

        // aruco_f -> cam_f
            // stiTf_Frame(tag_name,cam_name);
        // cam_f -> robot_f
            // stiTf_Frame(cam_name,robot_frame);
        // map -> robot_f
            // stiTf_Data(robotPose,map_frame,robot_frame);

        if(sub4 = true){
            sub4 = false ;
            int sl_tag = getPosearuco.detections.size() ;
            ROS_INFO("Detection : %d tag",sl_tag );
            for( int i = 0 ; i < sl_tag; i++ ){
                int id_tag = getPosearuco.detections.at(i).id.at(0);
                
                std::string id = to_string(id_tag);
                tag_name = tag_fistName + id ;
                ROS_INFO("frame= %s",tag_name.c_str());
                
                // map -> tag_f   
                    // stiBroadcaster( tagpose.at(id_tag-1) ,map_frame,listTag.at(id_tag-1));  
                    stiBroadcaster( mapTagPose.find(tag_name)-> second ,map_frame,tag_name);
                // tag_f -> cam_f
                    stiTf_Frame(tag_name,cam_name,robot_frame);
                // cam_f -> robot_f
                    // stiTf_Frame(cam_name,robot_frame);
                // map -> robot_f
                    // stiTf_Data(robotPose,map_frame,robot_frame);
                //pub
                poseRb.header.stamp = ros::Time::now();
                poseRb.header.frame_id = "map";
                poseRb.pose.pose.position.x       = robotPose.pose.position.x     ;                                 
                poseRb.pose.pose.position.y       = robotPose.pose.position.y     ;
                poseRb.pose.pose.position.z       = 0                             ;
                poseRb.pose.pose.orientation.x    = robotPose.pose.orientation.x  ;
                poseRb.pose.pose.orientation.y    = robotPose.pose.orientation.y  ;
                poseRb.pose.pose.orientation.z    = robotPose.pose.orientation.z  ;
                poseRb.pose.pose.orientation.w    = robotPose.pose.orientation.w  ;
                pub1.publish(poseRb);
                
                
            }
        } 

        



        
        r.sleep();
    }


}
