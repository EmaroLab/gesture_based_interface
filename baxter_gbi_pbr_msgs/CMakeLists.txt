cmake_minimum_required(VERSION 2.8.3)
project(baxter_gbi_pbr_msgs)

find_package(catkin
  REQUIRED
  COMPONENTS
  geometry_msgs
  nav_msgs
  message_generation
)

add_message_files(
   FILES
   playback_msg.msg
   record_status.msg
   mirror_end_effector.msg
   modify_playback.msg
)


generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
 )

catkin_package(
  CATKIN_DEPENDS
  nav_msgs
  geometry_msgs
  message_runtime
)

