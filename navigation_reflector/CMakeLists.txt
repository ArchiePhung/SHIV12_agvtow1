cmake_minimum_required(VERSION 3.0.2)
project(navigation_reflector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
  Raw_reflector.msg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)