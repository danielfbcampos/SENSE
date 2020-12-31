<p align="left">
<img src="cras.png" width="320" />
<img src="inesctec.png" width="320" />
</p>

# README #

* Name:SENSE/localization
* Rep: https://github.com/danielfbcampos/SENSE

### What is this repository for? ###

Localization module software repository of INESC TEC Centre for Robotics and Autonomous Systems.

### How do I set up? ###
   * Build and install libsbp

````
   cd mysrc       # cd to a directory where you will download and build libsbp (outside catkin workspace)
   git clone https://github.com/PaulBouchier/libsbp
   cd libsbp/c
   mkdir build
   cd build
   cmake ../
   make
   sudo make install   # install headers and libraries into /usr/local
````

   * Configure ROS environment: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
   * Install dependencies:
   
````
    sudo apt update
    sudo apt install ros-$ROS_DISTRO-robot-localization -y
````
   * Add the localization folder to your catkin workspace src folder
   * Compile catkin workspace

````
    cd catkin_ws       # your catkin workspace
    catkin_make
````

   * Run with:

````
    roslaunch launch_loc loc_box.launch
````

### Who do I talk to? ###

   * Andry Pinto ([andry.m.pinto@inestec.pt](mailto:andry.m.pinto@inestec.pt))
   * Daniel Campos ([daniel.f.campos@inestec.pt](mailto:daniel.f.campos@inestec.pt))

