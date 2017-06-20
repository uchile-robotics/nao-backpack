#!/bin/bash

echo "Installing NAO backpack related software..."
export CURR_PATH=$PWD
export CATKIN_WS=~/catkin_ws

mkdir -p ${CATKIN_WS}/src
cd ${CATKIN_WS}

# install general ros dependencies
sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-tf ros-kinetic-sensor-msgs -y
sudo apt-get install ros-kinetic-camera-calibration -y

# Add repositories
wstool init src ${CURR_PATH}/nao_install.rosinstall

# Install system and ROS dependencies
rosdep install -i -y --from-paths src

#catkin build --summarize -j1
catkin_make -j1

echo "source ${CATKIN_WS}/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
