/**
 * @file CognitionComm.h
 * This file declares the ROS interface to Cognition thread
 *
 * This file uses part of the BHuman Code Release 2016 <a href="https://github.com/bhuman/BHumanCodeRelease"</a>)
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */

#pragma once

// B-Human dependencies
#include "Tools/Communication/UdpComm.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/StreamHandler.h"
#include "Tools/Global.h"

// ROS Dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

// Other C++ dependencies
#include <unistd.h>
#include "CommBase.h"

namespace bhuman2ros
{

/** CognitionComm declaration */
class CognitionComm : public CommBase
{
    struct JoystickControl
    {
        float x_max_speed_;    // mm
        float x_min_speed_;    // mm
        float y_max_speed_;    // mm
        float rot_max_speed_;  // rad;
        float head_max_pan_;   // rad
        float head_min_tilt_;  // rad
        float head_max_tilt_;  // rad
    };

public:
    // Constructor
    CognitionComm() : CommBase() {}
    CognitionComm(std::string name, std::string ip, int port, int buffer_size, int rate, boost::mutex &mutex) : CommBase(name, ip, port, buffer_size, rate, mutex) {}

    // Methods
    void joystickToMotionRequest(const sensor_msgs::Joy::ConstPtr& joy);

    // Variables
    JoystickControl joystick_control_;
    boost::shared_ptr<image_transport::CameraPublisher> publisher_;


private:
    // Variables
    Image image_;       // The YUV422 image
    Image image_rgb_;   // RGB version

    // Joystick stuff
    MotionRequest motion_request_; // Motion request for joystick control
    HeadMotionRequest head_motion_request_; // Head motion request for joystick control
    bool should_kick_; // A flag to deactivate kicking after a kick
    bool control_omni_; // A flag to change the stick control from x-axis + rotation to x-axis + y-axis

    // Methods
    bool sendMessages();
    bool receiveMessages();
    bool handleMessage(InMessage &message);
    void prepareAndPublishImage();
    void fillImageRGB(Image& bh_image, sensor_msgs::Image &ros_image);
};

} //bhuman2ros
