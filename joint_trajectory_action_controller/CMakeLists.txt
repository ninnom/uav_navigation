cmake_minimum_required(VERSION 2.8.3)
project(joint_trajectory_action_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  trajectory_msgs
  actionlib_msgs
  std_msgs
  message_generation
  genmsg
)

find_package(Boost REQUIRED thread)

add_action_files(DIRECTORY action FILES MultiDofFollowJointTrajectory.action)

generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  actionlib_msgs
  joint_trajectory_action_controller
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
  CATKIN_DEPENDS message_runtime
  moveit_core
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
add_executable(joint_trajectory_action_controller src/joint_trajectory_action_controller.cpp)
target_link_libraries(joint_trajectory_action_controller ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

