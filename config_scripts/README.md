# Configuration Scripts

Configuration scripts allow you to configure the networking stuff between the NAO and the backpack. We assume that:

- Your NAO has fixed Ethernet IP
- Your backpack has a WiFi dongle

## Available scripts
### `hotspot.sh`
This enables a hotspot in the ODROID using a WiFi dongle.

Usage:

    ./hotspot.sh **SSID_name** on
    ./hotspot.sh **SSID_name** off


### `set_backpack_ip.sh`
This fixes the ethernet IP in the ODROID.

Usage:

    source set_backpack_ip.sh **IP_ethernet**


### `set_ros_master.sh`
Defines the variables `ROS_MASTER_URI` and `ROS_IP`. To be run in the ODROID or the laptop.

Usage:

    source set_ros_master.sh **ROS_MASTER_URI** **ROS_IP**


### `start_vnc.sh`
Runs the VNC server in the ODROID.

Usage:

    ./start_vnc.sh

## Running everything
We'll explain this with an example. In our setup we used the following configuration
**NAO:**
* Fixed Ethernet IP: 192.168.21.121. Configured within B-Human framework.

**Backpack (ODROID):**
* Fixed Ethernet IP: 192.168.21.200. Configured with `set_backpack_ip.sh`
* Hotspot with SSID `odroid-wifi` and WiFi IP 10.42.0.1 (default hotspot IP). Set with `hotspot.sh`
* `ROS_MASTER_URI=http://10.42.0.1:11311`, `ROS_IP=10.42.0.1`. Set with `set_ros_master.sh`

**Slave PC (for monitoring):**
* Connected to hotspot's network
* `ROS_MASTER_URI=http://10.42.0.1:11311`, `ROS_IP=10.42.0.X`. Set with `set_ros_master.sh` or manually.

This allowed us to connect the NAO and the backpack via Ethernet while monitoring ROS topics in a Slave PC via WiFi.
