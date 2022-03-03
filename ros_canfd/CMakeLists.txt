cmake_minimum_required(VERSION 3.0.2)
project(ros_canfd)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  can_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_canfd
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime can_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

#c++ include
add_executable(can_fd_dump src/can_fd_dump.cpp)
add_dependencies(can_fd_dump ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(can_fd_dump
  ${catkin_LIBRARIES}
 )

add_executable(can_fd_bridge src/can_fd_bridge.cpp)
add_dependencies(can_fd_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(can_fd_bridge
  ${catkin_LIBRARIES}
 )

