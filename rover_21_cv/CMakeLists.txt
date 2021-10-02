cmake_minimum_required(VERSION 2.8.3)
project(rover_21_cv)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  catkin
  roscpp
  rospy
  std_msgs
  tf
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   #std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rover_20_image
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include_directories(/usr/local/cuda-11.0/include)
  include_directories(/usr/local/zed/include)
)


add_executable(artag_detection src/artag_detection.cpp)
target_link_libraries(artag_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(aruco_slam src/aruco_slam.cpp)
target_link_libraries(aruco_slam ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(approaching_engine src/approaching_engine.cpp)
target_link_libraries(approaching_engine ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(aruco_tracking src/aruco_tracking.cpp)
target_link_libraries(aruco_tracking ${catkin_LIBRARIES} ${OpenCV_LIBS})
