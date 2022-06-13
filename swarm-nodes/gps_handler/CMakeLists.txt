cmake_minimum_required(VERSION 3.0.2)
project(gps_handler)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    swarm_msgs
)

catkin_package(
    CATKIN_DEPENDS message_generation rospy std_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/gps_handler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)