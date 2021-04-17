cmake_minimum_required(VERSION 2.8.11)
project(gr_control_gui)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS rviz roscpp visualization_msgs
                    geometry_msgs mongodb_store navigation_msgs
                    tf2_geometry_msgs std_msgs gr_map_utils nav_msgs)


#add_definitions(-Wno-deprecated-declarations)

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

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)

include_directories(${catkin_INCLUDE_DIRS})

add_library(offline_gui offline.cpp)
qt5_use_modules(offline_gui Widgets)
qt5_use_modules(offline_gui Core)
target_link_libraries(offline_gui ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES})

add_library(online_gui online.cpp)
qt5_use_modules(online_gui Widgets)
qt5_use_modules(online_gui Core)
target_link_libraries(online_gui ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES})

add_executable(online main_online.cpp)
qt5_use_modules(online Widgets)
qt5_use_modules(online Core)
target_link_libraries(online online_gui)

add_executable(offline main_online.cpp)
qt5_use_modules(offline Widgets)
qt5_use_modules(offline Core)
target_link_libraries(offline offline_gui)

# Instalv
install(TARGETS myviz_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
