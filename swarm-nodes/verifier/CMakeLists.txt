cmake_minimum_required(VERSION 3.0.2)
project(verifier)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros
  mavlink
  mavros_msgs
  custom_libs
)
	
generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs nav_msgs message_runtime
  #DEPENDS system_lib
  
)

include(MavrosMavlink)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include ${catkin_INCLUDE_DIRS} )

## Declare a C++ executable
add_executable(verifier src/verifier.cpp)
add_executable(collisionChecker src/collisionChecker.cpp)
add_executable(verifier_logger src/verifier_logger.cpp)

target_link_libraries(verifier ${catkin_LIBRARIES})
target_link_libraries(collisionChecker ${catkin_LIBRARIES})
target_link_libraries(verifier_logger ${catkin_LIBRARIES})

set_property(TARGET verifier PROPERTY CXX_STANDARD 20)
set_property(TARGET collisionChecker PROPERTY CXX_STANDARD 20)
set_property(TARGET verifier_logger PROPERTY CXX_STANDARD 20)