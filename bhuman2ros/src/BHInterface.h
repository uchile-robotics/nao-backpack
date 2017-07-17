/**
 * @file BHInterface.h
 * This file declares an UDP communication interface between a NAO and a external computer running ROS
 *
 * This file uses part of the BHuman Code Release 2016 <a href="https://github.com/bhuman/BHumanCodeRelease"</a>)
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */

#pragma once

// B-Human dependencies
#include "Tools/Streams/StreamHandler.h"
#include "Tools/Global.h"

// ROS Dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Joy.h>
//#include <geometry_msgs/PointStamped.h>

// Boost
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

// Other C++ dependencies
#include <unistd.h>

#include "CognitionComm.h"
#include "MotionComm.h"

namespace bhuman2ros {

class BHInterface
{
public:
    // Default Constructor
    BHInterface();

    // Default destructor
    ~BHInterface();

    // Joystick callback
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    // Print currently used parameters
    void printParameters();

private:
    // Node handle
    ros::NodeHandle node_handle_;

    // External parameters
    int rate_cognition_, rate_motion_;
    int port_cognition_, port_motion_;
    std::string ip_cognition_, ip_motion_;

    // ROS publishers and subscribers
    ros::Publisher imu_publisher_;
    ros::Publisher joint_state_publisher_;
    ros::Publisher fsr_publisher_;
    image_transport::CameraPublisher cam_publisher_;

    ros::Subscriber joy_subscriber_;

    // BHuman modules communication threads
    MotionComm* motion_comm_;
    CognitionComm* cognition_comm_;

    // Threads
    boost::thread* th_motion_;
    boost::thread* th_cognition_;

    // Mutex
    boost::mutex mutex_;

    // Stream Handler
    StreamHandler stream_handler_;
};

} //bhuman2ros
