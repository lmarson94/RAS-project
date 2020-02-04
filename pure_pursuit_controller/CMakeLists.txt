cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(pure_pursuit_controller)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  nav_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_action_files(
    DIRECTORY action
    FILES PurePursuit.action
)

generate_messages(
    DEPENDENCIES actionlib_msgs std_msgs geometry_msgs nav_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation roscpp std_msgs nav_msgs
)

add_executable(pure_pursuit_server src/pure_pursuit_server.cpp)
add_executable(pure_pursuit_client src/pure_pursuit_client.cpp)
add_executable(speedTest src/speedTest.cpp)

target_link_libraries(
  pure_pursuit_server
  ${catkin_LIBRARIES}
)

target_link_libraries(
  pure_pursuit_client
  ${catkin_LIBRARIES}
)

target_link_libraries(
  speedTest
  ${catkin_LIBRARIES}
)

add_dependencies(
  pure_pursuit_server pure_pursuit_generate_messages_cpp
  ${pure_pursuit_controller_EXPORTED_TARGETS}
)

add_dependencies(
  pure_pursuit_client pure_pursuit_generate_messages_cpp
  ${pure_pursuit_controller_EXPORTED_TARGETS}
)

add_dependencies(
  speedTest pure_pursuit_generate_messages_cpp
  ${pure_pursuit_controller_EXPORTED_TARGETS}
)
