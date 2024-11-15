cmake_minimum_required(VERSION 3.0.2)
project(message_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  roscpp
  rospy
)

add_message_files(
  DIRECTORY msg
  FILES
  Status_port.msg
  Status_launch.msg
  Velocities.msg
  Zone_safety.msg
  MC_status.msg
  Driver_query.msg
  Driver_respond.msg

  App_button.msg
  App_color.msg
  App_lbv.msg
  App_launch.msg

  Navigation_query.msg
  Navigation_respond.msg

  Parking_respond.msg
  Parking_request.msg

  CAN_received.msg
  CAN_send.msg
  CAN_status.msg

  CPD_read.msg
  CPD_write.msg

  Server_cmdRequest.msg

  Mecanum_request.msg
  Mecanum_respond.msg

  Toyo_input.msg
  Toyo_output.msg

  Funuc_control.msg
  Funuc_status.msg

  Try_control.msg
  Try_respond.msg
  Reconnect_sts.msg
  Status_reconnect.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES message_pkg
  CATKIN_DEPENDS message_runtime roscpp rospy
  # DEPENDS system_lib
)
