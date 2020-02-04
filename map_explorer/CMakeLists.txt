cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(map_explorer)

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
  nav_msgs
  roscpp
  occupancy_grid
  pure_pursuit_controller
)

add_action_files(
  DIRECTORY action
  FILES Exploration.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs nav_msgs geometry_msgs occupancy_grid
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_explorer
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation nav_msgs roscpp occupancy_grid pure_pursuit_controller
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(explorer_server src/explorer_server.cpp)
add_executable(explorer_client src/explorer_client.cpp)

target_link_libraries(
  explorer_server
  ${catkin_LIBRARIES}
)

target_link_libraries(
  explorer_client
  ${catkin_LIBRARIES}
)

#add_dependencies(
#  explorer_server
#  explorer_client
#  ${map_explorer_EXPORTED_TARGETS}
#)

add_dependencies(
  explorer_server map_explorer_generate_messages_cpp
  ${map_explorer_EXPORTED_TARGETS}
)

add_dependencies(
  explorer_client map_explorer_generate_messages_cpp
  ${map_explorer_EXPORTED_TARGETS}
)

