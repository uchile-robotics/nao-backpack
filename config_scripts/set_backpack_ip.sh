#!/usr/bin/env bash
# This file sets the backpack ethernet ip

echo "Setting backpack ethernet ip..."
if [ $# -gt 0 ]; then
        sudo ifconfig eth0 $1 netmask 255.255.255.0
else
        echo "No ip provided, using default: 192.168.21.100"
        sudo ifconfig eth0 192.168.21.100 netmask 255.255.255.0
fi
echo "Ethernet ip set to $1"
