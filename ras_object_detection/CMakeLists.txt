cmake_minimum_required(VERSION 2.8.3)
project(ras_object_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  object_detection
  tf
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   PointStampedArray.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
   geometry_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ras_motor_controller
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_position_estimation src/ObjectPositionEstimation.cpp)
target_link_libraries(object_position_estimation ${catkin_LIBRARIES} /opt/ros/kinetic/include/pcl)

find_package(catkin REQUIRED COMPONENTS tf)

add_dependencies(object_position_estimation ${catkin_EXPORTED_TARGETS})
