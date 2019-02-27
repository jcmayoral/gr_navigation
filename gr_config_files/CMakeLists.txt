cmake_minimum_required(VERSION 2.8.3)
project(gr_config_files)
find_package(catkin REQUIRED)

catkin_package()

### INSTALLS
install(DIRECTORY localization
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/localization
)

install(DIRECTORY navigation
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/navigation
)
