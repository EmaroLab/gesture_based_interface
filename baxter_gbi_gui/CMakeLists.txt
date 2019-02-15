cmake_minimum_required(VERSION 3.1)

project(baxter_gbi_gui)

set (CMAKE_CXX_STANDARD 17)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${PROJECT_SOURCE_DIR}/src/ui)

find_package(Qt5 REQUIRED COMPONENTS 
  Widgets
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmaster
  std_srvs
  sensor_msgs
  baxter_gbi_input_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rosmaster std_srvs sensor_msgs
)

add_executable(gui

  resources/resources.qrc

  src/general_utilities.cpp
  src/actionpanel.cpp
  src/configpanel.cpp
  src/main.cpp
  src/mainwindow.cpp
  src/mapping.cpp
  src/menupanel.cpp
  src/rosworker.cpp
  src/selectionfiltermodel.cpp
  src/tabcontent.cpp
  src/TriggerService.cpp
  src/BaxterDisplay.cpp
  src/TopicScanner.cpp
  src/FsmInputConfigurator.cpp

  src/include/general_utilities.h
  src/include/actionpanel.h
  src/include/configpanel.h
  src/include/mainwindow.h
  src/include/mapping.h
  src/include/menupanel.h
  src/include/rosworker.h
  src/include/selectionfiltermodel.h
  src/include/tabcontent.h
  src/include/TriggerService.h
  src/include/BaxterDisplay.h
  src/include/TopicScanner.h
  src/include/FsmInputConfigurator.h
  
  src/ui/actionpanel.ui
  src/ui/configpanel.ui
  src/ui/mainwindow.ui
  src/ui/mapping.ui
  src/ui/menupanel.ui
  src/ui/tabcontent.ui
)

target_compile_options(gui 
PRIVATE 
  -Wall -Wextra -pedantic)

target_include_directories(gui
PUBLIC
  ${catkin_INCLUDE_DIRS}
  src/include
)

target_link_libraries(gui
  Qt5::Widgets
  ${catkin_LIBRARIES}
)

add_dependencies(gui
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
