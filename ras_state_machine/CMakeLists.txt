cmake_minimum_required(VERSION 2.8.3)
project(ras_state_machine)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  rospy
  std_msgs
  std_srvs
  pure_pursuit_controller
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


#
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ras_state_machine
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation rospy std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  statemachine.py
  statemachie_MS3.py
  ${catkin_INCLUDE_DIRS}
)
