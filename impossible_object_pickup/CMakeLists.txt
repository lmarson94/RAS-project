cmake_minimum_required(VERSION 2.8.3)
project(impossible_object_pickup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  occupancy_grid
  visualization_msgs
  roscpp
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PickupImpossibleObject.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  nav_msgs
  occupancy_grid  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES impossible_object_pickup
#  CATKIN_DEPENDS geometry_msgs nav_msgs occupancy_grid roscpp
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(impossible_pickup_server src/impossible_pickup_server.cpp)
target_link_libraries(impossible_pickup_server ${catkin_LIBRARIES})
add_dependencies(impossible_pickup_server impossible_object_pickup_gencpp)

add_executable(impossible_pickup_client src/impossible_pickup_client.cpp)
target_link_libraries(impossible_pickup_client ${catkin_LIBRARIES})
add_dependencies(impossible_pickup_client impossible_object_pickup_gencpp)


