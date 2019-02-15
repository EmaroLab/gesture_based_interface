cmake_minimum_required(VERSION 2.8.3)
project(baxter_gbi_pbr)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  xacro
  actionlib
  sensor_msgs
  control_msgs
  geometry_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  baxter_core_msgs
  nav_msgs
  baxter_interface
  baxter_gbi_pbr_srv
  baxter_gbi_pbr_msg
  baxter_pykdl
)

catkin_python_setup()
 
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
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
  nav_msgs
  geometry_msgs
  baxter_interface
  baxter_gbi_pbr_srv
  baxter_gbi_pbr_msg
  baxter_pykdl
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

