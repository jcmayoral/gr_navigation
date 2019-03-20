cmake_minimum_required(VERSION 2.8.3)
project(gr_safety_monitors)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        roscpp
        fusion_msgs
        pluginlib
        fault_core
        )

find_package(OpenCV REQUIRED)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS} )


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES gr_safety_monitors
    CATKIN_DEPENDS
        dynamic_reconfigure
        roscpp
        fusion_msgs
        pluginlib
        fault_core
)

add_library(gr_safety_monitors src/proximity_monitor.cpp)
target_link_libraries(gr_safety_monitors ${catkin_LIBRARIES})

### INSTALLS
install(TARGETS gr_safety_monitors
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
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
