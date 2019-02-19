cmake_minimum_required(VERSION 2.8.3)
project(baxter_gbi_pbr_srvs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  message_generation
)

add_service_files(
  FILES
  CalibrateMirror.srv
  DeleteFile.srv
  EnableMirroring.srv
  Gripper.srv
  ListFiles.srv
  PauseResume.srv
  Playback.srv
  ReachGoal.srv
  RecordStart.srv
  RecordStop.srv
  RecordStop.srv
  RenameFile.srv
)

generate_messages(
  DEPENDENCIES
  baxter_gbi_pbr_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  baxter_gbi_pbr_msgs
  nav_msgs
  geometry_msgs
  message_runtime
)

