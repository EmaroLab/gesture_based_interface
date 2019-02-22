cmake_minimum_required(VERSION 2.8.3)
project(baxter_gbi_pbr_srvs)

find_package(catkin
  REQUIRED
  COMPONENTS
  geometry_msgs
  nav_msgs
  baxter_gbi_pbr_msgs
  message_generation
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
  GoToStart.srv
)


generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
   baxter_gbi_pbr_msgs
 )

catkin_package(
  CATKIN_DEPENDS
  nav_msgs
  geometry_msgs
  baxter_gbi_pbr_msgs
  message_runtime
)
