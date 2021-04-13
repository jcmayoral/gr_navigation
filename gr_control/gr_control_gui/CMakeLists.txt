cmake_minimum_required(VERSION 3.0.2)
project(gr_control_gui)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS rviz roscpp visualization_msgs
                    geometry_msgs mongodb_store navigation_msgs
                    tf2_geometry_msgs std_msgs gr_map_utils nav_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_definitions(-Wno-deprecated-declarations)

catkin_package(
    CATKIN_DEPENDS
      rviz
      roscpp
      visualization_msgs
      geometry_msgs
      mongodb_store
      navigation_msgs
      tf2_geometry_msgs
      std_msgs
      gr_map_utils
      nav_msgs
)

link_directories(${catkin_LIBRARY_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/myviz.cpp
  src/nodes_visualizer.hpp
  src/main.cpp
)

#add_library(${PROJECT_NAME}_lib src/myviz.cpp src/nodes_visualizer.hpp)
#target_link_libraries(${PROJECT_NAME}_lib ${QT_LIBRARIES} ${catkin_LIBRARIES})


## Add the "myviz" executable and specify the list of source files we
## collected above in ``${SRC_FILES}``.
add_executable(myviz ${SRC_FILES})

## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
target_link_libraries(myviz ${QT_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(myviz ${PROJECT_NAME}_lib)


## END_TUTORIAL

## Install
install(TARGETS myviz DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
