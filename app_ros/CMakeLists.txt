cmake_minimum_required(VERSION 3.0.2)
project(app_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sick_lidar_localization
)

# add_message_files(
# 	DIRECTORY msg
# 	FILES
#   App_button.msg
#   App_color.msg
#   App_lbv.msg
# )


include_directories(
  # include
    ${catkin_INCLUDE_DIRS}
  )
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs)