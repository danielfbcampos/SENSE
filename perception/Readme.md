<p align="left">
<img src="cras.png" width="320" />
<img src="inesctec.png" width="320" />
</p>

# README #

* Name:SENSE/perception
* Rep: https://github.com/danielfbcampos/SENSE

### What is this repository for? ###

Perception module software repository of INESC TEC Centre for Robotics and Autonomous Systems.

### How do I set up? ###
   * Build and install Mynt Eye D sdk (https://mynt-eye-d-sdk.readthedocs.io/en/latest/)

````
   cd mysrc       # cd to a directory where you will download and build mynt eye d sdk (outside catkin workspace)
   git clone https://github.com/slightech/MYNT-EYE-D-SDK.git
   cd MYNT-EYE-D-SDK
   make init
   make all
````

   * Configure ROS environment: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
   * Install dependencies:
   
````
    sudo apt update
    sudo apt install libpcap-dev -y
````

   * Add the perception folder to your catkin workspace src folder
   * Compile catkin workspace

````
    cd catkin_ws       # your catkin workspace
    catkin_make
````

   * Run with:

````
    roslaunch launch_perc perc.launch
````

### Who do I talk to? ###

   * Andry Pinto ([andry.m.pinto@inestec.pt](mailto:andry.m.pinto@inestec.pt))
   * Daniel Campos ([daniel.f.campos@inestec.pt](mailto:daniel.f.campos@inestec.pt))

