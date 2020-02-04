cmake_minimum_required(VERSION 2.8.3)
project(ras_localization)

find_package(catkin REQUIRED COMPONENTS
  phidgets
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
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

add_compile_options(-std=c++11)
add_executable(odometry src/Odometry.cpp)
add_executable(particle_filter src/ParticleFilter.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})
target_link_libraries(particle_filter ${catkin_LIBRARIES})
