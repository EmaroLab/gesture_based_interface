cmake_minimum_required(VERSION 2.8.3)
project(BaxterGBI_core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  roslib
  std_msgs
  std_srvs
  BaxterGBI_input_msgs
)

catkin_python_setup()

catkin_package(

  CATKIN_DEPENDS rospy smach std_msgs std_srvs BaxterGBI_input_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



