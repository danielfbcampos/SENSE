cmake_minimum_required(VERSION 2.8.3)
project(rc)


find_package(catkin REQUIRED COMPONENTS
  rospy serial geometry_msgs std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
   # INCLUDE_DIRS include
#    LIBRARIES test_mission
    CATKIN_DEPENDS rospy serial geometry_msgs std_msgs #custom_msgs nav_msgs sensor_msgs image_transport cv_bridge tf
   # DEPENDS OpenCV
)

catkin_install_python(PROGRAMS 
	src/rc.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

