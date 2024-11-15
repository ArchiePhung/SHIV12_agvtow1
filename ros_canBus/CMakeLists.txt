cmake_minimum_required(VERSION 3.0.2)
project(ros_canBus)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs roscpp rospy)

add_message_files(
	DIRECTORY msg
	FILES
  Status_conveyor.msg
  Control_conveyors.msg
  toyo_info.msg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs )

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs )