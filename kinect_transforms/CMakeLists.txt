cmake_minimum_required(VERSION 2.8.3)
project(kinect_transforms)

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
  roslaunch
  kinect_tracking_srvs
  tf
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
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime geometry_msgs kinect_tracking_srvs tf
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
add_executable(kinect_broadcaster src/kinect_broadcaster.cpp)
add_executable(tf_tracker src/tf_tracker.cpp)
add_executable(head_tracking src/head_tracking.cpp)
add_executable(kinect_to_baxter_broadcaster src/kinect_to_baxter_broadcaster.cpp)
add_executable(shadow_tf src/shadow_tf.cpp)
 
 ## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(head_tracking ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(kinect_broadcaster ${catkin_LIBRARIES})
target_link_libraries(tf_tracker ${catkin_LIBRARIES})
target_link_libraries(head_tracking ${catkin_LIBRARIES})
target_link_libraries(kinect_to_baxter_broadcaster ${catkin_LIBRARIES})
target_link_libraries(shadow_tf ${catkin_LIBRARIES})

target_compile_options(kinect_broadcaster PRIVATE -Wall -Wextra -pedantic)
target_compile_options(tf_tracker PRIVATE -Wall -Wextra -pedantic)
target_compile_options(head_tracking PRIVATE -Wall -Wextra -pedantic)
target_compile_options(kinect_to_baxter_broadcaster PRIVATE -Wall -Wextra -pedantic)
target_compile_options(shadow_tf PRIVATE -Wall -Wextra -pedantic)
