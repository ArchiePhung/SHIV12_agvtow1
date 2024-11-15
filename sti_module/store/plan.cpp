#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sti_msgs/Velocities.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/client.h>
// #include "param_base_localConfig.h"
// #include "param_costmap_commonConfig.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::PoseWithCovarianceStamped poseRb; 
sti_msgs::Velocities vel_robot  ;
move_base_msgs::MoveBaseGoal goal;
bool anable_aruco = false ;
bool vel_0 = false ;

void callSub1(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  poseRb = pose ;
  anable_aruco = true ;
}
void callSub2(const sti_msgs::Velocities& vel)
{
  vel_robot = vel ;
  if ( vel_robot.angular_z == 0) vel_0 = true ;
  else vel_0 = false ;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "plan");
    ros::NodeHandle n;

    //tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
    // sub - pub - client
        ros::Subscriber sub1 = n.subscribe("/posePoseRobotFromAruco", 100, callSub1);
        ros::Subscriber sub2 = n.subscribe("/raw_vel", 100, callSub2);
        ros::Publisher  setpost = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 50);
        ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

        // ros::ServiceClient client1 = n.serviceClient<std_srvs::Empty>("move_base/global_costmap/inflation_layer");

        // self.client1 = dynamic_reconfigure.client.Client("move_base/global_costmap/inflation_layer", timeout=30)
        // self.client1.update_configuration({"cost_scaling_factor":win_cost_scaling_factor,\
        //                                                    "inflation_radius":win_inflation_radius})

    // sec
        double secs1 = ros::Time::now().toSec() ;
        double secs2 = ros::Time::now().toSec() ;

        ros::Rate loop_rate(10);
        int buoc =1 ;

    move_base_msgs::MoveBaseGoal goal,goal21, goal22, goal41, goal42;
    // goal21
        goal21.target_pose.header.frame_id    = "map";
        goal21.target_pose.pose.position.x    = 1.825;
        goal21.target_pose.pose.position.y    = 1.356;
        goal21.target_pose.pose.orientation.z = -0.014;
        goal21.target_pose.pose.orientation.w = 1.000;
    // goal22
        goal22.target_pose.header.frame_id    = "map";
        goal22.target_pose.pose.position.x    = 2.550;
        goal22.target_pose.pose.position.y    = 1.344;
        goal22.target_pose.pose.orientation.z = -0.014;
        goal22.target_pose.pose.orientation.w = 1.000;
    // goal41
        goal41.target_pose.header.frame_id    = "map";
        goal41.target_pose.pose.position.x    = -10.203;
        goal41.target_pose.pose.position.y    = 2.461;
        goal41.target_pose.pose.orientation.z = 0.710;
        goal41.target_pose.pose.orientation.w = 0.705;
    // goal42
        goal42.target_pose.header.frame_id    = "map";
        goal42.target_pose.pose.position.x    = -10.265;
        goal42.target_pose.pose.position.y    = 3.192;
        goal42.target_pose.pose.orientation.z = 0.720;
        goal42.target_pose.pose.orientation.w = 0.693;

    // 
        std_srvs::Empty clearCostmap ;
    while (ros::ok())
    {   

        if ( buoc == 1 ){

            if (client.call(clearCostmap)){
                ROS_INFO("Clear Cosmap OK");

                goal = goal21 ;
                goal.target_pose.header.stamp = ros::Time::now();

                ac.sendGoal(goal);
                ac.waitForResult();
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO("goal 1 OK");
                    // buoc = 2 ;
                }

            }
            else  ROS_ERROR("Failed Clear Cosmap OK");
              
        }

        if ( buoc == 2 ){
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = -1.507;
            goal.target_pose.pose.position.y = 0.080;
            goal.target_pose.pose.position.z = 0.000;
            goal.target_pose.pose.orientation.x = 0;
            goal.target_pose.pose.orientation.y = 0;
            goal.target_pose.pose.orientation.z = 0.934;
            goal.target_pose.pose.orientation.w = 0.356;
            ac.sendGoal(goal);
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("goal 2 OK");
                buoc = 3 ;
            }              
        }

        if ( buoc == 3 ){
             ROS_INFO("huhu");
            if ( anable_aruco == true && vel_0 == true  ) {
                setpost.publish(poseRb);
                ROS_INFO("hihi");
                // sleep(3);
                ros::Duration(3.0).sleep();
                ros::spinOnce();
                ROS_INFO("set goal OK");
                buoc = 4 ;
            }
        }

        if ( buoc == 4 ){
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = -1.891;
            goal.target_pose.pose.position.y = 0.475;
            goal.target_pose.pose.position.z = 0.000;
            goal.target_pose.pose.orientation.x = 0;
            goal.target_pose.pose.orientation.y = 0;
            goal.target_pose.pose.orientation.z = 0.939;
            goal.target_pose.pose.orientation.w = 0.344;
            ac.sendGoal(goal);
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("goal 3 OK");
                // sleep(10);
                ros::Duration(10.0).sleep();
                ros::spinOnce();
                buoc = 5 ;
            }              
        }

        if ( buoc == 5 ){
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = -1.507;
            goal.target_pose.pose.position.y = 0.080;
            goal.target_pose.pose.position.z = 0.000;
            goal.target_pose.pose.orientation.x = 0;
            goal.target_pose.pose.orientation.y = 0;
            goal.target_pose.pose.orientation.z = 0.934;
            goal.target_pose.pose.orientation.w = 0.356;
            ac.sendGoal(goal);
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("goal 2 OK");                
                buoc = 1; //
            }             
        }

        
        
      

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

                //   secs1 =ros::Time::now().toSec();
                // if ( secs1 - secs2 > 0.5) {
                //     poseRb.header.frame_id = "map";
                //     setpost.publish(poseRb);
                //     secs2 = ros::Time::now().toSec() ; 
                // }