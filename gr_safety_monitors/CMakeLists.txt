cmake_minimum_required(VERSION 2.8.3)
project(gr_safety_monitors)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        roscpp
        pcl_ros
        fusion_msgs
        pluginlib
        fault_core
        visualization_msgs
        sensor_msgs
        std_msgs
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/ProximityMonitor.cfg
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES gr_safety_monitors
    CATKIN_DEPENDS
        dynamic_reconfigure
        roscpp
        pcl_ros
        fusion_msgs
        pluginlib
        fault_core
        visualization_msgs
        sensor_msgs
        std_msgs
)

add_library(gr_safety_monitors src/proximity_monitor.cpp  src/monitors_manager.cpp)
add_library(gr_safe_actions src/safe_actions/publisher_safe_action.cpp src/safe_actions/dynamic_reconfigure_safe_action.cpp)

add_executable(proximity_monitor_node src/proximity_monitor_node.cpp)
add_executable(monitors_manager_node src/monitors_manager_node.cpp)

target_link_libraries(gr_safety_monitors ${catkin_LIBRARIES}  ${PCL_LIBRARIES})
target_link_libraries(proximity_monitor_node gr_safety_monitors gr_safe_actions)

target_link_libraries(monitors_manager_node gr_safety_monitors gr_safe_actions)

add_dependencies(gr_safety_monitors ${catkin_EXPORTED_TARGETS} gr_safety_monitors_gencfg)
add_dependencies(proximity_monitor_node gr_safety_monitors gr_safe_actions gr_safety_monitors_gencfg)

add_dependencies(monitors_manager_node gr_safety_monitors gr_safe_actions gr_safety_monitors_gencfg)

### INSTALLS
install(TARGETS gr_safety_monitors
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS proximity_monitor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES monitors_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
