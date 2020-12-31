cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)

add_compile_options(-std=c++11)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    message_generation
    geometry_msgs
    std_msgs
    sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    FloatStamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    geometry_msgs
    sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)
