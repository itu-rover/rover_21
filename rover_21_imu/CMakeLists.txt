cmake_minimum_required(VERSION 2.8.3)
project(rover_21_imu)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(DIRECTORY cfg
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
	nodes/bosch_imu_node.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)
