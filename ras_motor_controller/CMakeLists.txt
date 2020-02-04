cmake_minimum_required(VERSION 2.8.3)
project(ras_motor_controller)
add_compile_options(-std=c++11)
set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS 1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  phidgets
  std_msgs
  geometry_msgs
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

add_executable(ras_motor_controller src/MotorController.cpp)
target_link_libraries(ras_motor_controller ${catkin_LIBRARIES})
