cmake_minimum_required(VERSION 2.8.3)
project(youbot_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  brics_actuator
  roscpp
  sensor_msgs
  geometry_msgs
  control_msgs
  tf
  control_toolbox
  actionlib
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs brics_actuator angles control_toolbox actionlib control_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(arm_position_control
src/code_base/youbot_arm_controller.cpp
src/ros_nodes/arm_position_control.cpp
)

target_link_libraries(arm_position_control
  ${catkin_LIBRARIES}
)

add_executable(manual_base_control
src/ros_nodes/manual_base_position_control.cpp
)

target_link_libraries(manual_base_control
  ${catkin_LIBRARIES}
)

add_executable(trajectory_test
src/ros_nodes/trajectory_test.cpp
)

target_link_libraries(trajectory_test
  ${catkin_LIBRARIES}
)

add_executable(base_position_controller
src/ros_nodes/base_position_controller.cpp
src/code_base/simple_base_position_controller.cpp
)

target_link_libraries(base_position_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(base_trajectory_action_server
src/ros_nodes/base_trajectory_action_server.cpp
src/code_base/base_trajectory_action.cpp
)

target_link_libraries(base_trajectory_action_server
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)