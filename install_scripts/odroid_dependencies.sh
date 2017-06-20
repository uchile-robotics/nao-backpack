#!/bin/bash

# Install odroid-utility.sh
echo "Installing odroid-utility.sh"
sudo -s
wget -O /usr/local/bin/odroid-utility.sh https://raw.githubusercontent.com/mdrjr/odroid-utility/master/odroid-utility.sh
chmod +x /usr/local/bin/odroid-utility.sh

# Install required libraries
echo "Installing general libraries"
sudo apt-get update
sudo apt-get install git cmake libboost-thread-dev -y

# Install VNC
echo "Installing VNC..."
sudo apt-get install x11vnc
echo "Creating password for VNC..."
x11vnc -storepasswd
