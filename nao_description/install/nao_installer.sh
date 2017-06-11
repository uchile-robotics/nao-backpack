#!/bin/bash

curr_path=$PWD
#mkdir -p ~/nao_ws/src
cd ~/catkin_ws

# Add repositories
wstool init src $curr_path/nao_install.rosinstall

# Install system and ROS dependencies
rosdep install -i -y --from-paths src

catkin build --summarize

echo "source ~/nao_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

./devel/tmp/installer.run --prefix ~/nao_ws/src/nao_meshes --mode unattended
