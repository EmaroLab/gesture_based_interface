cmake_minimum_required(VERSION 2.8.3)
project(kinect_tracking_srvs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   MoveKinect.srv
   RegulateKinectByWrist.srv
   RegulateKinectByHead.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
   std_msgs	
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

