cmake_minimum_required(VERSION 2.8.3)
project(ras_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  phidgets
  std_msgs
  tf
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

add_executable(broadcaster src/broadcaster.cpp)
target_link_libraries(broadcaster ${catkin_LIBRARIES})
find_package(catkin REQUIRED COMPONENTS tf)
