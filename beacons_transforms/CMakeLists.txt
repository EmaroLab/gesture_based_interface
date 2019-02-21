cmake_minimum_required(VERSION 2.8.3)
project(beacons_transforms)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  laser_geometry
  roslaunch
  tf
  visualization_msgs
  beacons_transforms_msgs
  beacons_transforms_srvs
)

roslaunch_add_file_check(launch)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  sensor_msgs 
  std_msgs 
  geometry_msgs 
  tf
  visualization_msgs
  beacons_transforms_msgs
  beacons_transforms_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare cpp executable
add_executable(beacon_broadcaster src/beacon_broadcaster.cpp)
 
add_dependencies(beacon_broadcaster beacons_transforms_generate_messages_cpp)

add_dependencies(beacon_broadcaster ${catkin_EXPORTED_TARGETS})	
target_link_libraries(beacon_broadcaster ${catkin_LIBRARIES})


## Specify libraries to link a library or executable target against
target_link_libraries(beacon_broadcaster ${catkin_LIBRARIES})
