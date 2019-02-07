cmake_minimum_required(VERSION 2.8.3)
project(pose_estimation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  laser_geometry
  pcl_ros
  
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

## Add include directories
include_directories(include
	  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES retrorov_model_bathymetric
 CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs message_runtime geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(retrorov_model_bathymetric
#   src/${PROJECT_NAME}/retrorov_model_bathymetric.cpp
# )

## Declare a cpp executable
add_executable(position_estimation src/position_estimation.cpp)
 
 ## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(position_estimation ${catkin_EXPORTED_TARGETS})	
		

## Specify libraries to link a library or executable target against
target_link_libraries(position_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

target_compile_options(position_estimation PRIVATE -Wall -Wextra -pedantic)
