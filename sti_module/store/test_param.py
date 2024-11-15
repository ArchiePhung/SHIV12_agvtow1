#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import dynamic_reconfigure.client

def setCapdo():
    rospy.init_node('myconfig_py', anonymous=True)
    
    # client2 = dynamic_reconfigure.client.Client("move_base/RotateRecovery", timeout=30)
    
    #Cap do 0 : Tay ps
    #Cap do 11 : lui ke
    # client1 = dynamic_reconfigure.client.Client("move_base", timeout=30)
    # client1.update_configuration({"base_global_planner":"stiplanner/StiPlanner",\
    #                               "base_local_planner" :"sti_localplanner/StiLocalPlanner",\
    #                               "planner_frequency":0,\
    #                               "recovery_behavior_enabled":"false",\
    #                               "oscillation_distance":0,\
    #                               "oscillation_timeout": 10,\
    #                               "max_planning_retries":10})

    # client2 = dynamic_reconfigure.client.Client("move_base/StiLocalPlanner", timeout=30)
    # client2.update_configuration({"tolerance_trans":0.1,\
    #                               "tolerance_rot" :0.1,\
    #                               "max_vel_lin":0.6,\
    #                               "max_vel_th":0.5})
    
    #Cap do 12 : di thang
    # client1 = dynamic_reconfigure.client.Client("move_base", timeout=30)
    # client1.update_configuration({"base_global_planner":"stiplanner/StiPlanner",\
    #                               "base_local_planner" :"dwa_local_planner/DWAPlannerROS",\
    #                               "planner_frequency":0,\
    #                               "recovery_behavior_enabled":"false",\
    #                               "oscillation_distance":0,\
    #                               "oscillation_timeout": 10,\
    #                               "max_planning_retries":10})

    # client2 = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS", timeout=30)
    # client2.update_configuration({"xy_goal_tolerance":0.1,\
    #                               "yaw_goal_tolerance" :0.1,\
    #                               "max_vel_x":0.6,\
    #                               "max_vel_theta":0.5})

    # Cap do 2
    # client1.update_configuration({"base_global_planner":"global_planner/GlobalPlanner"},\
    #                             {"base_local_planner" :"dwa_local_planner/DWAPlannerROS"},\
    #                             {"planner_frequency":10})
    
    # ekf
    client1 = dynamic_reconfigure.client.Client("ekf_localization", timeout=30)
    # client1.update_configuration({"imu0_config":"false, false, false,\
    #                                              false, false, false,\
    #                                              false, false, false,\
    #                                              false, false, false,\
    #                                              false, false, false"})
                                  
    print " Set cap do OK"


if __name__ == '__main__':
    try:
        setCapdo()
    except rospy.ROSInterruptException:
        pass