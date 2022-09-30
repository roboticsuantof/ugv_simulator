#!/bin/sh

## create a workspace
echo "\n\n CREATING WORKSPACE SOLAR_UGV_WS"
mkdir -p ~/solar_ugv_ws/src
cd ~/solar_ugv_ws/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH

echo "\n\n INSTALLING GAZEBO"
cd ~/solar_ugv_ws/src
git clone https://github.com/roboticsuantof/ugv_simulator.git

## Install Dependencies
sudo apt-get install ros-noetic-velodyne-gazebo-plugins ros-noetic-octomap-server


## Compile workspace
cd ~/solar_ugv_ws/ 
catkin_make
