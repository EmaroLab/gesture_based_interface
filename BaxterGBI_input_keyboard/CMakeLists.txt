cmake_minimum_required(VERSION 2.8.3)
project(BaxterGBI_input_keyboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  BaxterGBI_input_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy BaxterGBI_input_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



