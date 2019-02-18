cmake_minimum_required(VERSION 2.8.3)
project(baxter_gbi_core_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs
)

add_message_files(
   FILES
   status.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

