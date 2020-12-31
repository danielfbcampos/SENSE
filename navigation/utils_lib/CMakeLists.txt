cmake_minimum_required(VERSION 2.8.3)
project(utils_lib)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf
  tf_conversions
  laser_geometry)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

### PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_LIBRARIES}
)

catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES
  CATKIN_DEPENDS roscpp diagnostic_msgs pcl_ros pcl_conversions sensor_msgs tf tf_conversions laser_geometry
  #DEPENDS Eigen3
)

add_library(console_log src/console_log.cpp)
target_link_libraries(console_log ${catkin_LIBRARIES})


add_library(diagnostic_tools src/diagnostic_tools.cpp)
target_link_libraries(diagnostic_tools ${catkin_LIBRARIES})


add_library(utils_cras src/utils_cras.cpp)
target_link_libraries(utils_cras ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS console_log diagnostic_tools utils_cras
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

