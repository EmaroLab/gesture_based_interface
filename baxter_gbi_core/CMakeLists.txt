cmake_minimum_required(VERSION 2.8.3)
project(baxter_gbi_core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  roslib
  std_msgs
  std_srvs
  baxter_gbi_input_msgs
  baxter_gbi_pbr_msgs
  baxter_gbi_pbr_srvs
  genmsg actionlib_msgs
  actionlib
)

catkin_python_setup()


generate_messages(
   DEPENDENCIES actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy smach std_msgs std_srvs baxter_gbi_input_msgs
  baxter_gbi_pbr_msgs baxter_gbi_pbr_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



