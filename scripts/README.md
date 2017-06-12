# Configuration Scripts

Here we provide configuration scripts to facilitate some operations while using the backpack.

## `hotspot.sh`
This enables a hotspot in the ODROID using a WiFi dongle.

Usage:

    ./hotspot.sh **SSID_name** on
    ./hotspot.sh **SSID_name** off
    

## `set_backpack_ip.sh`
This fixes the ethernet IP in the ODROID.

Usage:

    source set_backpack_ip.sh **IP_ethernet**


## `set_ros_master.sh`
Defines the variables `ROS_MASTER_URI` and `ROS_IP`. To be run in the ODROID or the laptop.

Usage:

    source set_ros_master.sh **ROS_MASTER_URI** **ROS_IP**


## `start_vnc.sh`
Runs the VNC server in the ODROID.

Usage:

    ./start_vnc.sh



