#!/usr/bin/env bash
# This file configures ROS for hotspot connection

echo "Setting ROS_MASTER_URI..."
if [ $# -gt 0 ]; then
	# we use the argument as ROS_MASTER_URI
	export ROS_MASTER_URI=http://$1:11311/
	export ROS_IP=$2
else
	echo "No hostname provided, using hotspot's default: 10.42.0.1"
	export ROS_MASTER_URI=http://10.42.0.1:11311/
	export ROS_IP=10.42.0.1
fi
echo "ROS_MASTER_URI set to $ROS_MASTER_URI"
echo "ROS_IP set to $ROS_IP"
