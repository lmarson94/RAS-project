cmake_minimum_required(VERSION 2.8.3)
project(ras_obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ras_motor_controller
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_detection src/ObstacleDetection.cpp)
add_executable(lidar_detection src/LidarObstacleDetection.cpp)
target_link_libraries(obstacle_detection ${catkin_LIBRARIES})
target_link_libraries(lidar_detection ${catkin_LIBRARIES})
