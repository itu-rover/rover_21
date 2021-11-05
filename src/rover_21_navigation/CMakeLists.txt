cmake_minimum_required(VERSION 2.8.3)
project(rover_21_navigation)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslaunch
  sensor_msgs
  std_msgs
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

add_executable(path_following_node src/path_following_node.cpp)
target_link_libraries(path_following_node ${catkin_LIBRARIES} ${Eigen_LIBS})
