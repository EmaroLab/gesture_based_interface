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
  message_generation
  tf
  visualization_msgs
)

roslaunch_add_file_check(launch)

add_service_files(
   FILES
   MoveBeacon.srv
   SetRadius.srv
 )

add_message_files(
  FILES
  BeaconRadius.msg
)

generate_messages(
  DEPENDENCIES
   std_msgs
   geometry_msgs
   visualization_msgs
 )

catkin_package(
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime geometry_msgs tf visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare cpp executable
add_executable(beacon_broadcaster src/beacon_broadcaster.cpp)
 
add_dependencies(beacon_broadcaster beacons_transforms_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(beacon_broadcaster ${catkin_LIBRARIES})
