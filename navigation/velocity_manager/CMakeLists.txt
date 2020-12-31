cmake_minimum_required(VERSION 2.8.3)
project(velocity_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  custom_msgs
  nav_msgs
  sensor_msgs
  tf
  roboteq_msgs
  utils_lib
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
    include
    libs/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_LIBRARIES}
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include libs/include
    CATKIN_DEPENDS custom_msgs nav_msgs sensor_msgs tf roboteq_msgs utils_lib
    #DEPENDS Eigen3
)

###########
## Build ##
###########

add_library(kinematics libs/src/kinematics.cpp)
target_link_libraries(kinematics ${catkin_LIBRARIES}
                                 console_log
                                 ${Eigen3_LIBRARIES})

add_library(twist2thrust libs/src/twist2thrust.cpp)
target_link_libraries(twist2thrust ${catkin_LIBRARIES}
                                   kinematics
                                   console_log
                                   ${Eigen3_LIBRARIES}
                                   diagnostic_tools)

## Declare a C++ executable
add_executable(velocity_priority src/velocity_priority.cpp)

## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#set_target_properties(priority_driving PROPERTIES OUTPUT_NAME priority_driving PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(velocity_priority ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(velocity_priority ${catkin_LIBRARIES} diagnostic_tools)

## Declare a C++ executable
add_executable(twist2thrust_node src/twist2thrust_node.cpp)

## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(twist2thrust_node PROPERTIES OUTPUT_NAME twist2thrust PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(twist2thrust_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(twist2thrust_node ${catkin_LIBRARIES} twist2thrust)

#############
## Install ##
#############
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS velocity_priority kinematics twist2thrust twist2thrust_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/ libs/include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)
