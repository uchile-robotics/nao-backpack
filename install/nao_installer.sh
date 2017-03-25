#!/bin/bash

mkdir -p ~/nao_ws/src
cd ~/nao_ws

# Add repositories
wstool init src https://raw.githubusercontent.com/uchile-robotics/nao-backpack/master/install/nao_install.rosinstall?token=AK4UsvFQOYnmMAowv5z-eZjfauJLGGFRks5Y38mtwA%3D%3D

# Install system and ROS dependencies
rosdep install -i -y --from-paths src

catkin build --summarize

echo "source ~/nao_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

./devel/tmp/installer.run --prefix ~/nao_ws/src/nao_meshes --mode unattended