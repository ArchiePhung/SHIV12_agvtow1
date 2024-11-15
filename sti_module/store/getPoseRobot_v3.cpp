#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <sti_msgs/Setpose_control.h>
#include <sti_msgs/Setpose_status.h>
// file
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;
fstream f;
geometry_msgs::PoseStamped tag1,tag2;
vector<string> nameTag ;
vector<int> idTag ;
vector<geometry_msgs::PoseStamped> dataTag ;

apriltag_ros::AprilTagDetectionArray getPosearuco; 
geometry_msgs::PoseWithCovarianceStamped setposeRb; 
bool vel_0 = false ;
geometry_msgs::Pose positionRb ,poseloc;
bool sub4 = false ;
sti_msgs::Setpose_control sp_ctl ;
bool sub1 = false ;
bool sub2 = false ;
ros::Publisher pub2 ;


// call
    // 5hz
    void callSub1(const apriltag_ros::AprilTagDetectionArray& pose){ 
        getPosearuco = pose ; 
        if (sub1 == false ) sub1 = true ;
        // ROS_INFO("haha");
    }

    void callSub2(const sti_msgs::Setpose_control& data){
        sp_ctl = data ;
        if (sub2 == false ) sub2 = true ;
    }

    void callSub3(const nav_msgs::Odometry& data){
        if( data.twist.twist.linear.x == 0 ) vel_0 = true ;
        else vel_0 = false ;
    }

    void callSub4(const geometry_msgs::Pose& data){
        positionRb = data ;
        if (sub4 == false ) sub4 = true ;
    }

    void pub_status(int status, int id_tag, vector<int> listtag){
        sti_msgs::Setpose_status stt ;
        stt.status = status ;
        stt.find_tag = id_tag ;

        for(int i=0 ; i < listtag.size(); i++ ){
            stt.tagInFile.push_back( listtag.at(i) ) ;
        }

        pub2.publish(stt);
    }

    bool check_position(geometry_msgs::Pose p1 , geometry_msgs::Pose p2){
        if (fabs(p1.position.x - p2.position.x) < 0.02 && \
            fabs(p1.position.y - p2.position.y) < 0.02 && \
            fabs(p1.orientation.x - p2.orientation.x) < 0.02 && \
            fabs(p1.orientation.y - p2.orientation.y) < 0.02    )
        {
            return true ;
        }
        else return false ;
    }

// ham read file 
    void exportData(){ 

        // open file
        f.open("/home/stivietnam/catkin_ws/src/sti_module/data/data_tag.txt", ios::in);
        string data1,data2,data3,data4,data5,data6,data7;
        geometry_msgs::PoseStamped datTag ;
        string line, temp1,temp2;
        int dem = 0 ;
        while (!f.eof())
        {
            getline(f, line);
  
            temp1 = line; //-> get id
            temp2 = line; //-> get id
            line = line.substr(0, 5);
            if (line == "/tag_"){
                // get id
                dem ++ ;
                temp1 = temp1.substr(5,10);
                int id = stoi(temp1.c_str());
                
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
                datTag.header.frame_id    =   temp2 ;
                datTag.pose.position.x    =   stof(data1.c_str());
                datTag.pose.position.y    =   stof(data2.c_str());
                datTag.pose.position.z    =   stof(data3.c_str());
                datTag.pose.orientation.x =   stof(data4.c_str());
                datTag.pose.orientation.y =   stof(data5.c_str());
                datTag.pose.orientation.z =   stof(data6.c_str());
                datTag.pose.orientation.w =   stof(data7.c_str());

                // store
                idTag.push_back(id);
                nameTag.push_back(temp2);
                dataTag.push_back(datTag);

                // ROS_INFO("%d",idTag.at(dem -1)); 

                // ROS_INFO("%s : x= %f ,y= %f, z= %f", dataTag.header.frame_id.c_str(),\
                //                                         dataTag.pose.position.x,\
                //                                         dataTag.pose.position.y,\
                //                                         dataTag.pose.position.z);
                // ROS_INFO("%s : rx= %f ,ry= %f, rz= %f, rw= %f \n",dataTag.header.frame_id.c_str(),\
                //                                                     dataTag.pose.orientation.x,\
                //                                                     dataTag.pose.orientation.y,\
                //                                                     dataTag.pose.orientation.z,\
                //                                                     dataTag.pose.orientation.w);

            }            

        }
        f.close();
    }

int main(int argc, char** argv){
    // init
        ros::init(argc, argv, "getPoseRobotFromAruco");
        ros::NodeHandle n;
        ros::Time current_time, last_vel_time;
        ros::Rate r(100.0);

    // sub - pub
        
        ros::Publisher  pub1    = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("posePoseRobotFromAruco", 100);
        ros::Publisher  setpost = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 50);
        pub2 = n.advertise<sti_msgs::Setpose_status>("setpose_status",100);
        ros::Publisher  pub3    = n.advertise<std_msgs::String>("syscommand", 100);

        ros::Subscriber subscribe1    = n.subscribe("/tag_detections", 100, callSub1);
        ros::Subscriber subscribe2 = n.subscribe("/setpose_control",100,callSub2);
        ros::Subscriber subscribe3 = n.subscribe("/raw_odom",100,callSub3);
        ros::Subscriber subscribe4 = n.subscribe("/robot_pose",100,callSub4);
        
    // TF
        geometry_msgs::TransformStamped trans1, trans2, trans3, trans4;
        tf::TransformBroadcaster broadcaster1, broadcaster2, broadcaster3, broadcaster4;

        tf::TransformListener listener1,listener2,listener3,listener4;
        tf::StampedTransform transform1,transform2,transform3,transform4;
        // listener1.waitForTransform("/map", tag_name, ros::Time(), ros::Duration(1.0));
    // sec
        double secs1 =ros::Time::now().toSec();
        double secs2 = ros::Time::now().toSec() ;
        
    // bien
        int buoc = -1 ;
        int idTarget = 0 ;
        int idTagFind = 0 ;
        int dem_filter = 0 ;
        int sl_filter = 10 ;
        geometry_msgs::PoseStamped dataTarget ;

    // string
        string map_frame,tag_name,tag_fistName,cam_name, robot_frame;
        string f_cam_name ,f_tag_name ,f_tag_fistName ,f_robot_frame;
        map_frame      = "/map";
        tag_fistName   = "/tag_";
        cam_name       = "/camera_aruco";
        robot_frame    = "/base_footprint";

        f_cam_name     = "/f_camera_aruco";   
        f_tag_fistName = "/f_tag_";
        f_robot_frame  = "/f_robot";


    // read file
        exportData();

        ROS_INFO("detection : %d Tag",int(idTag.size()));
        for (int i=0; i<idTag.size() ; i++){
            ROS_INFO("id = %d",idTag.at(i));

            ROS_INFO("%s : x= %f ,y= %f, z= %f",dataTag.at(i).header.frame_id.c_str(),\
                                                dataTag.at(i).pose.position.x,\
                                                dataTag.at(i).pose.position.y,\
                                                dataTag.at(i).pose.position.z);
            ROS_INFO("%s : rx= %f ,ry= %f, rz= %f, rw= %f \n",dataTag.at(i).header.frame_id.c_str(),\
                                                            dataTag.at(i).pose.orientation.x,\
                                                            dataTag.at(i).pose.orientation.y,\
                                                            dataTag.at(i).pose.orientation.z,\
                                                            dataTag.at(i).pose.orientation.w);
        }

        pub_status(-1,0,idTag);

    while(n.ok()){
        ros::spinOnce();
        pub_status(buoc,idTagFind,idTag);

        //B-1 :wait cmt
        if (buoc == -1){
            ROS_INFO("buoc: %d", buoc);
            ROS_WARN("wait cmt");
            pub_status(buoc,idTagFind,idTag);
            if (sub2 == true && sp_ctl.setposeTag != 0){
                sub2 = false ;
                idTarget = sp_ctl.setposeTag ;
                tag_name = tag_fistName + to_string(idTarget) ;
                f_tag_name = f_tag_fistName + to_string(idTarget) ;
                buoc = 1 ;
                ROS_INFO("buoc: %d", buoc);
                pub_status(buoc,idTagFind,idTag);        
            }
            
        }

        //B1 :check id tag in file 
        // and get dataTarget
        if( buoc == 1){
            ROS_WARN("nhan lenh Ok , check file");
            bool check = false ;
            for (int i=0; i<idTag.size() ; i++){
                if ( idTarget == idTag.at(i)){ // khop tag trong file
                    dataTarget = dataTag.at(i);
                    check = true ;
                }
            }
            if (check == true ){
                buoc = 2 ;
                ROS_WARN("check tag in cam");
            }
            else {
                buoc = -2 ;
                ROS_WARN("khong co data trong file");
            }
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
        }
        
        //B-2
        if (buoc == -2 ){
            ROS_WARN("k co data trong file");
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
            if(sp_ctl.setposeTag == 0){
                buoc = -1 ;
            }
        }

        //B2 :check id tag in camera
        if (buoc == 2){
            ROS_WARN("check id tag in camera");
            if ( sub1 == true){
                sub1 = false ;
                int sl_tag = getPosearuco.detections.size() ;             
                if ( sl_tag != 0){
                    idTagFind = getPosearuco.detections.at(0).id.at(0) ;
                    
                    if (idTarget == idTagFind){
                        buoc = 3 ; // khop tag
                        ROS_WARN("khop tag in camera");
                    }
                    else {
                        buoc = -3 ; // khong khop tag in camera
                        ROS_WARN("khong khop tag in camera");
                    }
                }
                else{
                    buoc = -4 ; // k nhin thay tag
                    ROS_WARN("k nhin thay tag");
                }
            }
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
        }

        //B-3 :khong khop tag in camera
        if (buoc == -3){
            ROS_WARN("khong khop tag in camera");
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
            if(sp_ctl.setposeTag == 0){
                buoc = -1 ;
            }
        }

        //B-4 : k nhin thay tag
        if (buoc == -4){
            ROS_WARN("k nhin thay tag");
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
            if(sp_ctl.setposeTag == 0){
                buoc = -1 ;
            }
        }

        //B3 :check k/c
        if (buoc ==3){
            ROS_WARN("check k/c");
            float kc_x = getPosearuco.detections.at(0).pose.pose.pose.position.x ;
            float kc_z = getPosearuco.detections.at(0).pose.pose.pose.position.z ;
            float kc = sqrt(kc_x*kc_x + kc_z*kc_z) ;

            if (fabs(kc) < sp_ctl.distance ){
                buoc = 4 ;
                ROS_WARN("set pose");
            }
            else{
                buoc = -5 ;
                ROS_WARN("k/x qua xa");
            }
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
        }

        //B-5 :k/x qua xa
        if (buoc == -5 ){
            ROS_WARN("k/x qua xa");
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
            if(sp_ctl.setposeTag == 0){
                buoc = -1 ;
            }
        }

        // khop tag -> tranform : /map-> /tag -> /cam -> /robot
        // loc + check vel_0 = true
        if (buoc == 4){
            pub_status(buoc,idTagFind,idTag);
            
            // map-> tag
                trans1.header.stamp = ros::Time::now();
                trans1.header.frame_id      = map_frame;
                trans1.child_frame_id       = f_tag_name;
                trans1.transform.translation.x = dataTarget.pose.position.x  ;                                         
                trans1.transform.translation.y = dataTarget.pose.position.y  ;
                trans1.transform.translation.z = dataTarget.pose.position.z  ;
                trans1.transform.rotation.x    = dataTarget.pose.orientation.x ;
                trans1.transform.rotation.y    = dataTarget.pose.orientation.y ;
                trans1.transform.rotation.z    = dataTarget.pose.orientation.z ; 
                trans1.transform.rotation.w    = dataTarget.pose.orientation.w ; 
                broadcaster1.sendTransform(trans1);

            // tf : // tag -> cam
                listener2.lookupTransform(tag_name, cam_name, ros::Time(0), transform2);
                trans2.header.stamp = ros::Time::now();
                trans2.header.frame_id = f_tag_name;
                trans2.child_frame_id  = f_cam_name;
                trans2.transform.translation.x = transform2.getOrigin().x() ;                                          
                trans2.transform.translation.y = transform2.getOrigin().y() ;
                trans2.transform.translation.z = transform2.getOrigin().z() ;
                trans2.transform.rotation.x    = transform2.getRotation().x() ;
                trans2.transform.rotation.y    = transform2.getRotation().y() ;
                trans2.transform.rotation.z    = transform2.getRotation().z() ; 
                trans2.transform.rotation.w    = transform2.getRotation().w() ; 
                broadcaster2.sendTransform(trans2);

            // tf :cam -> robot
                listener3.lookupTransform(cam_name, robot_frame, ros::Time(0), transform3);
                trans3.header.stamp = ros::Time::now();
                trans3.header.frame_id = f_cam_name;
                trans3.child_frame_id  = f_robot_frame;
                trans3.transform.translation.x = transform3.getOrigin().x() ;                                          
                trans3.transform.translation.y = transform3.getOrigin().y() ;
                trans3.transform.translation.z = transform3.getOrigin().z() ;
                trans3.transform.rotation.x    = transform3.getRotation().x() ;
                trans3.transform.rotation.y    = transform3.getRotation().y() ;
                trans3.transform.rotation.z    = transform3.getRotation().z() ; 
                trans3.transform.rotation.w    = transform3.getRotation().w() ; 
                broadcaster3.sendTransform(trans3);
        

            // tf : map -> robot
                listener4.waitForTransform(map_frame, f_robot_frame, ros::Time(), ros::Duration(1));
                try{
                    listener4.lookupTransform(map_frame, f_robot_frame, ros::Time(0), transform4);
                    trans4.header.stamp = ros::Time::now();
                    trans4.header.frame_id = map_frame;
                    trans4.child_frame_id =  "f_rb";
                    trans4.transform.translation.x = transform4.getOrigin().x() ;                                          
                    trans4.transform.translation.y = transform4.getOrigin().y() ;
                    trans4.transform.translation.z = transform4.getOrigin().z() ;
                    trans4.transform.rotation.x    = transform4.getRotation().x() ;
                    trans4.transform.rotation.y    = transform4.getRotation().y() ;
                    trans4.transform.rotation.z    = transform4.getRotation().z() ; 
                    trans4.transform.rotation.w    = transform4.getRotation().w() ; 
                    broadcaster4.sendTransform(trans4);

                    // check v=0
                    if ( vel_0 == true){
                        
                        poseloc.position.x     += transform4.getOrigin().x() ;                                          
                        poseloc.position.y     += transform4.getOrigin().y() ;
                        poseloc.orientation.z  += transform4.getRotation().z() ; 
                        poseloc.orientation.w  += transform4.getRotation().w() ; 

                        dem_filter ++ ;
                        ROS_WARN("dem_filter= %d",dem_filter);
                    }
                    if (dem_filter == sl_filter ){
                        poseloc.position.x    = poseloc.position.x / sl_filter ; 
                        poseloc.position.y    = poseloc.position.y / sl_filter ; 
                        poseloc.orientation.x = poseloc.orientation.x / sl_filter ; 
                        poseloc.orientation.y = poseloc.orientation.y / sl_filter ; 
                        
                        std_msgs::String clearMap ;
                        clearMap.data = "reset";
                        pub3.publish(clearMap);
                        buoc = 5 ;
                    } 
                    if (dem_filter > sl_filter ){
                        dem_filter = 0 ;
                        poseloc.position.x = 0 ;
                        poseloc.position.y = 0 ; 
                        poseloc.orientation.z = 0 ;
                        poseloc.orientation.w = 0 ;
                        buoc = 4 ;
                    }
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                }
        
        }

        // setpose  + check position 
        if (buoc == 5){
            ROS_WARN("setpose  + check position ");
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);

            setposeRb.header.stamp = ros::Time::now();
            setposeRb.header.frame_id = "map";
            setposeRb.pose.pose.position.x       = poseloc.position.x     ;                                 
            setposeRb.pose.pose.position.y       = poseloc.position.y     ;
            setposeRb.pose.pose.orientation.z    = poseloc.orientation.z  ;
            setposeRb.pose.pose.orientation.w    = poseloc.orientation.w  ;
            if ( vel_0 == true){
                setpost.publish(setposeRb);
            }
            else buoc = 5 ; 

            if ( sub4 == true){
                sub4 = false ;
                if ( check_position(poseloc,positionRb) == true) buoc = 6 ;
                else buoc = 5 ;
            }

        }

        if (buoc == 6){
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
            ROS_WARN("DONE!!");
            if(sp_ctl.setposeTag == 0){
                dem_filter = 0 ;
                poseloc.position.x = 0 ;
                poseloc.position.y = 0 ;
                poseloc.orientation.z = 0 ;
                poseloc.orientation.w = 0 ;
                buoc = -1 ;
            }
        }


 
        r.sleep();
    }
}
