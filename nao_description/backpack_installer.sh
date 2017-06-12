#!/bin/bash

export CURR_PATH=$PWD
echo ${CURR_PATH}
export CATKIN_WS=~/catkin_ws
echo ${CATKIN_WS}

#mkdir -p ~/nao_ws/src
cd ${CATKIN_WS}

# Add repositories
wstool init src ${CURR_PATH}/nao_install.rosinstall

# Install system and ROS dependencies
rosdep install -i -y --from-paths src

#catkin build --summarize -j1
catkin_make -j1

echo "source ${CATKIN_WS}/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

#chmod +x ${CATKIN_WS}/devel/tmp/installer.run
#sudo .${CATKIN_WS}/devel/tmp/installer.run --prefix ${CATKIN_WS}/src/nao_meshes --mode unattended
