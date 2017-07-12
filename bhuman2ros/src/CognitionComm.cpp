/**
 * @file CognitionComm.cpp
 * This file implements the ROS interface to Cognition thread
 *
 * This file uses part of the BHuman Code Release 2016 <a href="https://github.com/bhuman/BHumanCodeRelease"</a>)
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */

#include "CognitionComm.h"

namespace bhuman2ros{

bool CognitionComm::sendMessages()
{
    bool success = false;

    out_queue_.out.bin << motion_request_;
    out_queue_.out.finishMessage(idMotionRequest);

    out_queue_.out.bin << head_motion_request_;
    out_queue_.out.finishMessage(idHeadMotionRequest);

#ifdef VERBOSE
    ROS_INFO("Cognition Comm: Sending %d messages", out_queue_.getNumberOfMessages());
#endif

    // send messages
    if(out_queue_.getNumberOfMessages())
    {
        int size = out_queue_.getStreamedSize();

        char* buffer = 0;
        buffer = new char[size];
        OutBinaryMemory stream(buffer);
        stream << out_queue_;

        success = socket_.write(buffer, size);

#ifdef VERBOSE
    ROS_INFO("Cognition Comm: Sending was successful? %d", success);
#endif

        delete [] buffer;
    }
    out_queue_.clear();

  return success;
}

bool CognitionComm::receiveMessages()
{
    if(!port_)
        return 0; // not started yet

    int tmp_size = 0;
    int m_size = 0;
    int expected_size;

    expected_size = 14400000;

    // fill the message buffer with the packets
    while(m_size < expected_size  && ros::ok())
    {
        tmp_size = socket_.read(tmp_buf_, m_buf_size_);
        if(tmp_size > 0)
        {
            // if the packets are delayed, it is possible to receive the initial packet with the image size
            // in that case, we reset the current image and we start again
            if(tmp_size == sizeof(unsigned int))
            {
                m_size = 0;
                expected_size = ((unsigned int*) tmp_buf_)[0];
                continue;
            }
            // otherwise, we assume the packets are correct and we keep saving them
            else
            {
                memcpy( & m_buf_[m_size], tmp_buf_, tmp_size);
                m_size += tmp_size;
            }
        }
    }
    InBinaryMemory memory(&m_buf_[0], m_size);
    memory >> queue_;

    return queue_.getNumberOfMessages() > 0;
}

bool CognitionComm::handleMessage(InMessage &message)
{
    switch(message.getMessageID())
    {
    case idImage:
    {
        message.bin >> image_;
        prepareAndPublishImage();
        return true;
    }
    default:
        return false;
    }

    return false;
}

void CognitionComm::prepareAndPublishImage()
{
#ifdef VERBOSE
    ROS_INFO("Cognition Comm: Publishing Image!");
#endif

    // convert image to RGB
    image_rgb_.convertFromYCbCrToRGB(image_);

    // declare ROS image and camera info
    sensor_msgs::Image ros_image;
    sensor_msgs::CameraInfo ros_cam_info;

    // convert to ROS format
    fillImageRGB(image_rgb_, ros_image);

    // fill camera info
    ros_cam_info.height = image_rgb_.height;
    ros_cam_info.width = image_rgb_.width;
    ros_cam_info.header.stamp = time;
    ros_cam_info.header.frame_id = "/CameraTop_optical_frame";

    // publish image
    publisher_->publish(ros_image, ros_cam_info);
}

void CognitionComm::fillImageRGB(Image& bh_image, sensor_msgs::Image &ros_image)
{
    ros_image.encoding = "rgb8";
    ros_image.height   = image_rgb_.height;
    ros_image.width    = image_rgb_.width;
    ros_image.step     = 3 * bh_image.width;

    size_t st0 = (ros_image.step * ros_image.height);
    ros_image.data.resize(st0);
    ros_image.is_bigendian = 0;

    for(int i=0; i<bh_image.height; i++)
        for(int j=0; j<bh_image.width; j++)
        {
            ros_image.data[3*(i*bh_image.width + j)    ] = image_rgb_[i][j].r;
            ros_image.data[3*(i*bh_image.width + j) + 1] = image_rgb_[i][j].g;
            ros_image.data[3*(i*bh_image.width + j) + 2] = image_rgb_[i][j].b;
        }
}

void CognitionComm::joystickToMotionRequest(const sensor_msgs::Joy::ConstPtr& joy)
{
    // BUTTONS
    // 0 - A - walk
    if(joy->buttons[0])
    {
        motion_request_.motion = MotionRequest::walk;
    #ifdef VERBOSE
            ROS_INFO("Cognition Comm: MotionRequest: WALK");
    #endif
    }
    // 1 - B - stand
    if(joy->buttons[1])
    {
        motion_request_.motion = MotionRequest::stand;
    #ifdef VERBOSE
            ROS_INFO("Cognition Comm: MotionRequest: STAND");
    #endif
    }
    // 2 - X - kick - TODO: implement
    if(joy->buttons[2])
    {
        motion_request_.motion = MotionRequest::kick;
        should_kick_ = true;
    #ifdef VERBOSE
            ROS_INFO("Cognition Comm: MotionRequest: KICK");
    #endif
    }
    // 3 - Y - special action
    if(joy->buttons[3])
    {
        motion_request_.motion = MotionRequest::specialAction;
    #ifdef VERBOSE
            ROS_INFO("Cognition Comm: MotionRequest: SPECIAL ACTION");
    #endif
    }
    // 6 - back
    if(joy->buttons[6])
    {
        control_omni_ = control_omni_? false : true;
    }

    // fill head motion request
    head_motion_request_.cameraControlMode = HeadMotionRequest::upperCamera;
    head_motion_request_.pan  = joystick_control_.head_max_pan_ * joy->axes[3];
    head_motion_request_.tilt =  -joy->axes[4] * (joy->axes[4] >= 0.f? joystick_control_.head_max_tilt_ : joystick_control_.head_min_tilt_);
    head_motion_request_.speed = 2.f;
#ifdef VERBOSE
        ROS_INFO("Cognition Comm: HeadMotionRequest: pan: %f, tilt: %f", head_motion_request_.pan, head_motion_request_.tilt);
#endif

    // avoid kicking again
    if(!should_kick_ && motion_request_.motion == MotionRequest::kick)
        motion_request_.motion = MotionRequest::specialAction;

    // analyze by case
    switch(motion_request_.motion)
    {
        case MotionRequest::walk:
            motion_request_.walkRequest.mode = WalkRequest::speedMode;

            if(!control_omni_)
            {
                // stick case: the stick controls x-axis linear + rotational motion
                motion_request_.walkRequest.speed.translation.x() = joy->axes[1] * (joy->axes[1]>=0.f? joystick_control_.x_max_speed_ : joystick_control_.x_min_speed_);
                motion_request_.walkRequest.speed.translation.y() = 0.f;
                motion_request_.walkRequest.speed.rotation = Angle(joystick_control_.rot_max_speed_ * joy->axes[0]);
            }
            else
            {
                // cross case: omnidirectional walking without rotation
                motion_request_.walkRequest.speed.translation.x() = joy->axes[1] * (joy->axes[1]>=0.f? joystick_control_.x_max_speed_ : joystick_control_.x_min_speed_);
                motion_request_.walkRequest.speed.translation.y() = joy->axes[0] * joystick_control_.y_max_speed_;
                motion_request_.walkRequest.speed.rotation = 0.f;
            }
            #ifdef VERBOSE
                ROS_INFO("Cognition Comm: Motion Request: x: %f, y: %f, rot: %f", motion_request_.walkRequest.speed.translation.x(), motion_request_.walkRequest.speed.translation.y(), (float)motion_request_.walkRequest.speed.rotation);
            #endif
            break;

        case MotionRequest::specialAction:
            motion_request_.specialActionRequest.specialAction = SpecialActionRequest::standHigh;
        #ifdef VERBOSE
            ROS_INFO("Cognition Comm: MotionRequest: SPECIAL ACTION: Stand High");
        #endif
            break;

        case MotionRequest::kick:
            motion_request_.kickRequest.kickMotionType = KickRequest::kickForward;
            should_kick_ = false;
        #ifdef VERBOSE
            ROS_INFO("Cognition Comm: MotionRequest: KICK");
        #endif
            break;

        default:
            motion_request_.specialActionRequest.specialAction = SpecialActionRequest::standHigh;
        #ifdef VERBOSE
            ROS_INFO("Cognition Comm: MotionRequest: SPECIAL ACTION: Stand High");
        #endif
            break;
    }
}

} //bhuman2ros
