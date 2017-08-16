# bhuman2ros-node

This ROS node implements an UDP-based communication bridge between the B-Human framework and the ROS middleware to publish the images, joints and sensors from the NAO.
It is based on the [B-Human coderelease 2016](https://github.com/bhuman/BHumanCodeRelease/releases/tag/coderelease2016), since this is the version used by [UChile Robotics Team](http://uchilert.amtc.cl).

This also includes a simple interface to control the NAO using an XBox controller based on the ROS' joy node.

## Required libraries
- [ROS](http://www.ros.org)
- Boost (1.5.4)
- `nao_robot`, `nao_meshes`, `naoqi_driver` and `naoqi_bridge` packages. Please follow the instructions in the `nao_description` folder.
- `joy` package to control the NAO with a joystick.
- `bhuman_modules` included in the nao-backpack package. It is mandatory to install these modules in you NAO in order to use this bridge.
- `naoqi_bridge_msgs` ROS package. We use this to publish FSR sensors

## Usage
We provide a launch file to run everything you need in you backpack/laptop to get the NAO data.

    roslaunch bhuman2ros nao_bringup.launch

It has 5 options:
* `version` (default: V40) : NAO description version 
* `use_rviz` (default: false) : Run RViz to visualize data
* `use_joy` (default: false) : Run the joystick node
* `joy_dev` (default: /dev/input/js0) : Select the joystick device
* `robot_ip` (default: 192.168.21.121) : IP of the robot connected to the backpack via Ethernet.

## License
Please see the NAO Backpack package license.

## Troubleshooting
* **The data is not being published in ROS**: Please check that you are subscribing the right topics (by deault, all are published under the `bhuman2ros` namespace).
* **The joystick is not working**: Even the data can be published and displayed in ROS, it is possible that MotionRequests and HeadMotionRequests are not send back to NAO. Please check that you are using the correct robot IP.
* **The NAO code fails on run time**: If your are experiencing some errors to run the NAO software while using the backpack, please check that your `MessageID.h` file in your B-Human folder is exactly the same as the backpack's. Since this file is basically an enumeration of message identifiers, any difference will cause inconsistencies in message handling between the NAO and the backpack.

## Pending work
* The current joystick control is hardcoded and based on an XBox 360 controller
* The kick behaviors are not implemented yet


