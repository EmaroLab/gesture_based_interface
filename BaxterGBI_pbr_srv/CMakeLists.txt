cmake_minimum_required(VERSION 2.8.3)
project(BaxterGBI_pbr_srv)

find_package(catkin
  REQUIRED
  COMPONENTS
  geometry_msgs
  nav_msgs
  BaxterGBI_pbr_msg
)


add_service_files(
  FILES
  Playback.srv
  RecordStart.srv
  RecordStop.srv
  ListFiles.srv
  DeleteFile.srv
  RenameFile.srv
  Gripper.srv
  ReachGoal.srv
  CalibrateMirror.srv
  EnableMirroring.srv
  PauseResume.srv
)


generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
   BaxterGBI_pbr_msg
 )

catkin_package(
  CATKIN_DEPENDS
  nav_msgs
  geometry_msgs
  BaxterGBI_pbr_msg
)
