cmake_minimum_required(VERSION 2.8.3)
project(pose_publisher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp tf)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_pose_publisher src/robot_pose_publisher.cpp)
target_link_libraries(robot_pose_publisher
  ${catkin_LIBRARIES}
)
install(TARGETS robot_pose_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(robot_pose_v1 src/robot_pose_v1.cpp)
target_link_libraries( robot_pose_v1
  ${catkin_LIBRARIES}
)
install(TARGETS robot_pose_v1
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)