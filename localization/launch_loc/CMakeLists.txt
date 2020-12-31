cmake_minimum_required(VERSION 2.8.3)
project(launch_pkg)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY cfg    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
