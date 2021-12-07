cmake_minimum_required(VERSION 3.0.2)
project(arm_op_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_modules
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_op_control
  CATKIN_DEPENDS roscpp rospy
  #DEPENDS system_lib
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(moveit_functions
    include/moveit_functions.h
    src/moveit_functions.cpp
)
target_link_libraries(moveit_functions ${catkin_LIBRARIES})
add_dependencies(moveit_functions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(joy_functions
    include/joy_functions.h
    src/joy_functions.cpp
)

target_link_libraries(joy_functions ${catkin_LIBRARIES})
add_dependencies(joy_functions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_library(autonomous_functions
    include/autonomous_functions.h
    src/autonomous_functions.cpp
)

target_link_libraries(autonomous_functions ${catkin_LIBRARIES})
add_dependencies(autonomous_functions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(arm_control src/arm_control.cpp)
add_dependencies(arm_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_control
${catkin_LIBRARIES})

#add_library(moveit_functions src/moveit_functions.cpp)
target_link_libraries(arm_control moveit_functions joy_functions autonomous_functions)
#target_link_libraries(test_custom joy_functions)

