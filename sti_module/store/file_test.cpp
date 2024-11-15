#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
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

// ham read file 

    void exportData(){ 

        // open file
        f.open("/home/stivietnam/catkin_ws/src/vector_convert/data/data_tag.txt", ios::in);
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

    
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "file");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    
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

   
    while (ros::ok()){

       
        // ros::spinOnce();
        ros::spin();
        loop_rate.sleep();

    }
    
    return 0;
}