cmake_minimum_required(VERSION 3.1)

project(BaxterGBI_gui)

set (CMAKE_CXX_STANDARD 17)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

#set(Qt5_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5")

find_package(Qt5 REQUIRED Widgets)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmaster
  std_msgs
  BaxterGBI_input_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES BaxterGBI_gui
  CATKIN_DEPENDS roscpp rosmaster std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
)

file (
  GLOB project_SOURCES 
  "src/*.cpp" 
  "src/*.ui"
)

file (
  GLOB project_HEADERS 
  "include/BaxterGBI_gui/*.h"
)

add_executable(
  gui 
  ${project_SOURCES} 
  ${project_HEADERS}
)

target_link_libraries(
  gui 
  Qt5::Widgets 
  ${catkin_LIBRARIES}
)

add_dependencies(
  gui 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
