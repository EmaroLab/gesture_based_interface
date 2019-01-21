cmake_minimum_required(VERSION 2.8.3)
project(BaxterGBI_pbr)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  xacro
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  baxter_core_msgs
  baxter_interface
)

catkin_python_setup()
 
#add_message_files(
#   FILES
#   playback_msg.msg
#)

add_service_files(
  FILES
  Playback.srv
  Record.srv
  ListFiles.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS
  rospy
  xacro
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  baxter_core_msgs
  baxter_interface
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
