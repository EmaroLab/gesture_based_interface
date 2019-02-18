cmake_minimum_required(VERSION 2.8.3)
project(beacons_presence_scanner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  roslaunch
)

roslaunch_add_file_check(launch)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES retrorov_model_bathymetric
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs 
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(beacons_scanner src/beacons_scanner.cpp)
 
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(beacons_scanner ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(beacons_scanner ${catkin_LIBRARIES})

