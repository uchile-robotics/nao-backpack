/**
 * @file BHInterface.cpp
 * This file implements an UDP communication interface between a NAO and a external computer running ROS
 *
 * This file uses part of the BHuman Code Release 2016 <a href="https://github.com/bhuman/BHumanCodeRelease"</a>)
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */

#include "BHInterface.h"
#include <iostream>

#define IMG_BUF_SIZE 700000
#define BUF_SIZE 4096
#define PACKET_SIZE 1400

namespace bhuman2ros{

BHInterface::BHInterface() : node_handle_("~")
{
    // Read communication parameters from parameter server
    node_handle_.param("rate_cognition", rate_cognition_, 50);
    node_handle_.param("rate_motion", rate_motion_, 200);

    node_handle_.param("port_cognition", port_cognition_, 10040);
    node_handle_.param("port_motion", port_motion_, 10030);

    node_handle_.param("ip_cognition", ip_cognition_, std::string("127.0.0.1"));
    node_handle_.param("ip_motion", ip_motion_, std::string("127.0.0.1"));

#ifdef VERBOSE
    printParameters(); // Print parameters
#endif

    // Callbacks
    joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&BHInterface::joyCallback, this, _1));

    // Create communication interfaces
    // Sensors
    motion_comm_ = new MotionComm("MotionComm", ip_motion_, port_motion_, BUF_SIZE, rate_motion_, mutex_);

    imu_publisher_ = node_handle_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
    fsr_publisher_ = node_handle_.advertise<geometry_msgs::PointStamped>("fsr", 1);
    fsr_l_publisher_ = node_handle_.advertise<naoqi_bridge_msgs::FloatArrayStamped>("fsr_l", 1);
    fsr_r_publisher_ = node_handle_.advertise<naoqi_bridge_msgs::FloatArrayStamped>("fsr_r", 1);
    fsr_l_total_publisher_ = node_handle_.advertise<naoqi_bridge_msgs::FloatStamped>("fsr_l_total", 1);
    fsr_r_total_publisher_ = node_handle_.advertise<naoqi_bridge_msgs::FloatStamped>("fsr_r_total", 1);
    joint_state_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 1);

    motion_comm_->imu_publisher_    = boost::shared_ptr<ros::Publisher>(&imu_publisher_);
    motion_comm_->joints_publisher_ = boost::shared_ptr<ros::Publisher>(&joint_state_publisher_);
    motion_comm_->fsr_publisher_    = boost::shared_ptr<ros::Publisher>(&fsr_publisher_);
    motion_comm_->fsr_l_publisher_  = boost::shared_ptr<ros::Publisher>(&fsr_l_publisher_);
    motion_comm_->fsr_r_publisher_  = boost::shared_ptr<ros::Publisher>(&fsr_r_publisher_);
    motion_comm_->fsr_l_total_publisher_  = boost::shared_ptr<ros::Publisher>(&fsr_l_total_publisher_);
    motion_comm_->fsr_r_total_publisher_  = boost::shared_ptr<ros::Publisher>(&fsr_r_total_publisher_);

    // Cognition
    image_transport::ImageTransport it(node_handle_);
    cam_publisher_  = it.advertiseCamera("camera/image_raw", 1);
    cognition_comm_ = new CognitionComm("CognitionComm", ip_cognition_, port_cognition_, IMG_BUF_SIZE, rate_cognition_, mutex_);
    cognition_comm_->publisher_ = boost::shared_ptr<image_transport::CameraPublisher>(&cam_publisher_);

    // Read joystick parameters from the parameter server
    node_handle_.param("head_max_pan",  cognition_comm_->joystick_control_.head_max_pan_,   2.1f);
    node_handle_.param("head_max_tilt", cognition_comm_->joystick_control_.head_max_tilt_,  0.5f);
    node_handle_.param("head_min_tilt", cognition_comm_->joystick_control_.head_min_tilt_,  0.5f);
    node_handle_.param("x_max_speed",   cognition_comm_->joystick_control_.x_max_speed_,   200.f);
    node_handle_.param("x_min_speed",   cognition_comm_->joystick_control_.x_min_speed_,   150.f);
    node_handle_.param("y_max_speed",   cognition_comm_->joystick_control_.y_max_speed_,   200.f);
    node_handle_.param("rot_max_speed", cognition_comm_->joystick_control_.rot_max_speed_,  0.5f);

    // Launch threads
    th_motion_     = new boost::thread( boost::bind(&MotionComm::main, motion_comm_) );
    th_cognition_  = new boost::thread( boost::bind(&CognitionComm::main, cognition_comm_) );
}

BHInterface::~BHInterface()
{
    if(th_motion_)
    {
        th_motion_->join();
        delete th_motion_;
    }
    if(th_cognition_)
    {
        th_cognition_->join();
        delete th_cognition_;
    }
}

void BHInterface::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    cognition_comm_->joystickToMotionRequest(joy);
}

void BHInterface::printParameters()
{
    std::cout << "Initializing bhuman2ros node" << std::endl;
    std::cout << "rate_cognition: " <<  rate_cognition_ << "\n"
              << "rate_motion: "    <<  rate_motion_ << "\n"
              << "port_cognition: " <<  port_cognition_ << "\n"
              << "port_motion: "    <<  port_motion_ << "\n"
              << "ip_cognition: "   <<  ip_cognition_ << "\n"
              << "ip_motion: "      <<  ip_motion_ << std::endl;
}

} //bhuman2ros
