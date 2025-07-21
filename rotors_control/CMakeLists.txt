cmake_minimum_required(VERSION 2.8.3)
project(rotors_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES twist_controller lee_position_controller roll_pitch_yawrate_thrust_controller generic_pid
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(generic_pid src/library/generic_pid.cpp)
target_link_libraries(generic_pid ${catkin_LIBRARIES})
add_dependencies(generic_pid ${catkin_EXPORTED_TARGETS})

add_library(twist_controller src/library/twist_controller.cpp)
target_link_libraries(twist_controller ${catkin_LIBRARIES})
add_dependencies(twist_controller ${catkin_EXPORTED_TARGETS})

add_library(lee_position_controller src/library/lee_position_controller.cpp)
target_link_libraries(lee_position_controller ${catkin_LIBRARIES})
add_dependencies(lee_position_controller ${catkin_EXPORTED_TARGETS})

add_library(roll_pitch_yawrate_thrust_controller src/library/roll_pitch_yawrate_thrust_controller.cpp)
target_link_libraries(roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})
add_dependencies(roll_pitch_yawrate_thrust_controller ${catkin_EXPORTED_TARGETS})


add_executable(twist_controller_node src/nodes/twist_controller_node.cpp)
add_dependencies(twist_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_controller_node
  twist_controller generic_pid ${catkin_LIBRARIES})

add_executable(lee_position_controller_node src/nodes/lee_position_controller_node.cpp)
add_dependencies(lee_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lee_position_controller_node
  lee_position_controller ${catkin_LIBRARIES})

add_executable(roll_pitch_yawrate_thrust_controller_node
  src/nodes/roll_pitch_yawrate_thrust_controller_node.cpp)
add_dependencies(roll_pitch_yawrate_thrust_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(roll_pitch_yawrate_thrust_controller_node
  roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})

install(TARGETS twist_controller lee_position_controller roll_pitch_yawrate_thrust_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS twist_controller_node lee_position_controller_node roll_pitch_yawrate_thrust_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
