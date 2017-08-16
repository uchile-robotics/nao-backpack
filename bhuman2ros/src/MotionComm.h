/**
 * @file MotionComm.h
 * This file declares the ROS interface to Motion thread
 *
 * This file uses part of the BHuman Code Release 2016 <a href="https://github.com/bhuman/BHumanCodeRelease"</a>)
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */

#pragma once

// B-Human dependencies
#include "Tools/Communication/UdpComm.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/StreamHandler.h"
#include "Tools/Global.h"

// ROS Dependencies
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <naoqi_bridge_msgs/FloatArrayStamped.h>
#include <naoqi_bridge_msgs/FloatStamped.h>

// Boost
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

// Other dependencies
#include <unistd.h>
#include "CommBase.h"

namespace bhuman2ros {

class MotionComm : public CommBase
{
public:
    // Variables
    boost::shared_ptr<ros::Publisher> imu_publisher_;
    boost::shared_ptr<ros::Publisher> joints_publisher_;
    boost::shared_ptr<ros::Publisher> fsr_publisher_;
    boost::shared_ptr<ros::Publisher> fsr_r_publisher_, fsr_r_total_publisher_;
    boost::shared_ptr<ros::Publisher> fsr_l_publisher_, fsr_l_total_publisher_;

    // Constructor
    MotionComm() : CommBase() {}
    MotionComm(std::string name, std::string ip, int port, int buffer_size, int rate, boost::mutex &mutex) : CommBase(name, ip, port, buffer_size, rate, mutex) {}

private:
    // Internal variables
    InertialSensorData inertial_sensor_data_;
    FsrSensorData fsr_sensor_data_;
    JointSensorData joint_sensor_data_;
    SystemSensorData system_sensor_data_;
    JointAngles joint_angles_;

    float fsr_l_;
    float fsr_r_;

    // Methods
    bool sendMessages() { return true;}
    bool receiveMessages();
    void configurePublisher(ros::NodeHandle& node_handle);
    bool handleMessage(InMessage &message);

    void prepareAndPublishIMU();
    void prepareAndPublishFSR();
    void prepareAndPublishJointSensorData();
    void prepareAndPublishSystemData();
    void prepareAndPublishJoints();

};

} //bhuman2ros
