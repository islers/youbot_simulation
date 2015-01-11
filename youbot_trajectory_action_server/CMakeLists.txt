cmake_minimum_required(VERSION 2.8.3)
project(youbot_trajectory_action_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  brics_actuator
  control_msgs
  orocos_kdl
  roscpp
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES youbot_trajectory_action_server
  CATKIN_DEPENDS actionlib brics_actuator control_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(youbot_trajectory_action_server
    src/joint_trajectory_action.cpp
)