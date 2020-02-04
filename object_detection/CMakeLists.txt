cmake_minimum_required(VERSION 2.8.3)
project(object_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  message_generation
) 

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   msg_objects_detected.msg
 )


## Generate services in the 'srv' folder
add_service_files(
   FILES
   DetectObjectsImage.srv
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_detection
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
   object_detector.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
