#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
    
using namespace std;
fstream f;
geometry_msgs::PoseStamped tag1,tag2;
char key;

// ham write file 
    void writeData(geometry_msgs::PoseStamped& tag_data){ 
        
        string tag_name = *(&tag_data.header.frame_id) ;
        string datax  = to_string(*(&tag_data.pose.position.x));
        string datay  = to_string(*(&tag_data.pose.position.y));
        string dataz  = to_string(*(&tag_data.pose.position.z));
        string datarx = to_string(*(&tag_data.pose.orientation.x));
        string datary = to_string(*(&tag_data.pose.orientation.y));
        string datarz = to_string(*(&tag_data.pose.orientation.z));
        string datarw = to_string(*(&tag_data.pose.orientation.w));


        f << "\n" + tag_name;
        f << "\n    x= " + (string)datax;       
        f << "\n    y= " + (string)datay;
        f << "\n    z= " + (string)dataz;
        f << "\n   rx= " + (string)datarx;    
        f << "\n   ry= " + (string)datary;       
        f << "\n   rz= " + (string)datarz;
        f << "\n   rw= " + (string)datarw;

    }

    // For non-blocking keyboard inputs
    int getch(void)
    {
        int ch;
        struct termios oldt;
        struct termios newt;

        // Store old settings, and copy to new settings
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        // Make required changes and apply the settings
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_iflag |= IGNBRK;
        newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
        newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;
        tcsetattr(fileno(stdin), TCSANOW, &newt);

        // Get the current character
        ch = getchar();

        // Reapply old settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        return ch;
    }

    
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "write_file");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    
    tag1.header.frame_id    = "tag_1" ; 
    tag1.pose.position.x    =  -0.121212121 ;
    tag1.pose.position.y    =  1.23434344 ;
    tag1.pose.position.z    =  1.3 ;
    tag1.pose.orientation.x =  1.4 ;
    tag1.pose.orientation.y =  1.5 ;
    tag1.pose.orientation.z =  1.6 ;
    tag1.pose.orientation.w =  1.7 ;
    
    f.open("/home/stivietnam/catkin_ws/src/vector_convert/data/data_tag.txt", ios::out);
    printf("\n\n                 go EMTER de luu ARUCO              \n\n");
    int dem =0; 
    while (ros::ok()){

        key = getch();

        if(key ==13){
        //     
            writeData(tag1);   
            ROS_INFO("da luu : %s",tag1.header.frame_id.c_str());
            dem ++;

        // 
        }

     
        if (key == '\x03')
        {   
            
            printf("\n\n                 Da luu data in file : data_tag.txt              \n\n");
            break;
        }
        
          
        
        ros::spinOnce();
        loop_rate.sleep();

    }
    f.close();
    return 0;
}