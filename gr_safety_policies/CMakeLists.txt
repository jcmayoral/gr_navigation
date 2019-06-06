cmake_minimum_required(VERSION 2.8.3)
project(gr_safety_policies)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        roscpp
        pcl_ros
        pluginlib
        safety_core
        visualization_msgs
        sensor_msgs
        geometry_msgs
        std_msgs
        cv_bridge
        message_filters
)

find_package(OpenCV 3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/ProximityPolicy.cfg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES gr_safety_policies
    CATKIN_DEPENDS
        dynamic_reconfigure
        roscpp
        pcl_ros
        pluginlib
        safety_core
        visualization_msgs
        sensor_msgs
        geometry_msgs
        std_msgs
        cv_bridge
        message_filters
)

file(GLOB POLICIES src/policies/*.cpp)
file(GLOB ACTIONS src/safe_actions/*.cpp)

add_library(gr_safety_policies ${POLICIES} src/policies_manager.cpp)
add_library(gr_safe_actions ${ACTIONS})

add_executable(proximity_policy_node src/proximity_policy_node.cpp)
add_executable(depth_proximity_policy_node src/depth_proximity_policy_node.cpp)
add_executable(weather_policy_node src/weather_policy_node.cpp)
add_executable(policies_manager_node src/policies_manager_node.cpp)
add_executable(action_manager_node src/action_manager_node.cpp)

add_definitions(-DCURL_STATICLIB)

target_link_libraries(gr_safety_policies ${catkin_LIBRARIES} curl ${JSON_INCLUDE_DIRS} ${OpenCV_LIBRARIES})
#Independent policies
target_link_libraries(proximity_policy_node gr_safety_policies gr_safe_actions)
target_link_libraries(depth_proximity_policy_node gr_safety_policies gr_safe_actions)
target_link_libraries(weather_policy_node gr_safety_policies gr_safe_actions)
#Policies Manager
target_link_libraries(policies_manager_node gr_safety_policies gr_safe_actions)
#Action Manager
target_link_libraries(action_manager_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(gr_safety_policies ${catkin_EXPORTED_TARGETS} gr_safety_policies_gencfg)
add_dependencies(proximity_policy_node gr_safety_policies gr_safe_actions gr_safety_policies_gencfg)
add_dependencies(depth_proximity_policy_node gr_safety_policies gr_safe_actions gr_safety_policies_gencfg)
add_dependencies(weather_policy_node gr_safety_policies gr_safe_actions gr_safety_policies_gencfg)
add_dependencies(policies_manager_node gr_safety_policies gr_safe_actions gr_safety_policies_gencfg)

### INSTALLS
install(TARGETS gr_safety_policies gr_safe_actions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS depth_proximity_policy_node proximity_policy_node weather_policy_node 
                policies_manager_node action_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES policies_plugins.xml action_policies.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
