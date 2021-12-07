cmake_minimum_required(VERSION 2.8.3)
project(rover_21_navigation)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslaunch
  sensor_msgs
  message_generation
  std_msgs
)
add_message_files(
  DIRECTORY msg FILES 
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)




include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_package()

find_package(Eigen3 REQUIRED)

roslaunch_add_file_check(launch)

install(
  DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

