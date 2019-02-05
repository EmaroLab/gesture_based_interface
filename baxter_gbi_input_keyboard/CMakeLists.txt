cmake_minimum_required(VERSION 2.8.3)
project(baxter_gbi_input_keyboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  baxter_gbi_input_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy baxter_gbi_input_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



