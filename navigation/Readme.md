<p align="left">
<img src="cras.png" width="320" />
<img src="inesctec.png" width="320" />
</p>

# README #

* Name:SENSE/navigation
* Rep: https://github.com/danielfbcampos/SENSE

### What is this repository for? ###

Navigation module software repository of INESC TEC Centre for Robotics and Autonomous Systems.

### How do I set up? ###

   * Configure ROS environment: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
   * Install dependencies:
   
````
    sudo apt update
    sudo apt install ros-$ROS_DISTRO-robot-localization  ros-$ROS_DISTRO-serial -y

````
   * Add the navigation folder to your catkin workspace src folder
   * Compile catkin workspace

````
    cd catkin_ws       # your catkin workspace
    catkin_make

````

### Who do I talk to? ###

   * Andry Pinto ([andry.m.pinto@inestec.pt](mailto:andry.m.pinto@inestec.pt))
   * Daniel Campos ([daniel.f.campos@inestec.pt](mailto:daniel.f.campos@inestec.pt))

