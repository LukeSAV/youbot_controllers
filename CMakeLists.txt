cmake_minimum_required(VERSION 2.8.3)
project(youbot_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  brics_actuator
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs brics_actuator
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(arm_position_control
src/youbot_arm_controller.cpp
src/arm_position_control.cpp
)

target_link_libraries(arm_position_control
  ${catkin_LIBRARIES}
)