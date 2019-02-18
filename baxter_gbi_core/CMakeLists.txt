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
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy smach std_msgs std_srvs baxter_gbi_input_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



