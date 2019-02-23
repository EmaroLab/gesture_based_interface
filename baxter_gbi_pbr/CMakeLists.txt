cmake_minimum_required(VERSION 2.8.3)
project(baxter_gbi_pbr)

find_package(catkin REQUIRED COMPONENTS
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
  roslaunch
  baxter_interface
  baxter_gbi_pbr_srvs
  baxter_gbi_pbr_msgs
  baxter_pykdl
  genmsg
  actionlib_msgs
)

roslaunch_add_file_check(launch)
catkin_python_setup()
 

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
  baxter_gbi_pbr_srvs
  baxter_gbi_pbr_msgs
  baxter_pykdl
  actionlib_msgs
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
