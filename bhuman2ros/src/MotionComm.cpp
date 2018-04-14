/**
 * @file MotionComm.cpp
 * This file implements the ROS interface to Motion thread
 *
 * This file uses part of the BHuman Code Release 2016 <a href="https://github.com/bhuman/BHumanCodeRelease"</a>)
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */

#include "MotionComm.h"
#include <iostream>

namespace bhuman2ros
{

bool MotionComm::receiveMessages()
{
    if(!port_)
        return 0; // not started yet

    int m_size = 0;

    // Read socket
    do{
        m_size = socket_.read(tmp_buf_, tmp_buf_size_);
        if(m_size > 0)
        {
            InBinaryMemory memory(tmp_buf_, m_size);
            memory >> queue_;

            #ifdef VERBOSE
                ROS_INFO("Motion Comm: packet received of size %d", m_size);
            #endif
        }
    } while(m_size > 0  && ros::ok());

    return queue_.getNumberOfMessages() > 0;
}

bool MotionComm::handleMessage(InMessage &message)
{
    switch(message.getMessageID())
    {
    case idFsrSensorData:
        message.bin >> fsr_sensor_data_;
        prepareAndPublishFSR();
        return true;

    case idInertialSensorData:
        message.bin >> inertial_sensor_data_;
        prepareAndPublishIMU();
        return true;

    case idJointSensorData:
        message.bin >> joint_sensor_data_;
        prepareAndPublishJointSensorData();
        return true;

    case idSystemSensorData:
        message.bin >> system_sensor_data_;
        prepareAndPublishSystemData();
        return true;

    case idJointAngles:
        message.bin >> joint_angles_;
        prepareAndPublishJoints();
        return true;
    default:
        return false;
    }
    return false;
}

void MotionComm::prepareAndPublishIMU()
{
#ifdef VERBOSE
    ROS_INFO("Publishing IMU!");
#endif

    // Fill IMU
    sensor_msgs::Imu ros_imu;
    ros_imu.header.stamp = time_;
    ros_imu.header.frame_id = "/ImuTorsoGyrometer_frame";

    // fill orientation
    tf::Quaternion q;
    q.setRPY(inertial_sensor_data_.angle(0), inertial_sensor_data_.angle(1), -inertial_sensor_data_.angle(2));  // a patch to fix the inverted yaw angle
    ros_imu.orientation.x = q.x();
    ros_imu.orientation.y = q.y();
    ros_imu.orientation.z = q.z();
    ros_imu.orientation.w = q.w();
    //rosIMU.orientation = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    //ROS_INFO("IMU: x: %f, y: %f, z: %f", inertial_sensor_data_.angle(0).toDegrees(), inertial_sensor_data_.angle(1).toDegrees(), inertial_sensor_data_.angle(2).toDegrees());

    // fill gyro
    ros_imu.angular_velocity.x = inertial_sensor_data_.gyro(0);
    ros_imu.angular_velocity.y = inertial_sensor_data_.gyro(1);
    ros_imu.angular_velocity.z = inertial_sensor_data_.gyro(2);
    //rosIMU.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    // fill accelerometer
    ros_imu.linear_acceleration.x = inertial_sensor_data_.acc(0);
    ros_imu.linear_acceleration.y = inertial_sensor_data_.acc(1);
    ros_imu.linear_acceleration.z = inertial_sensor_data_.acc(2);
    //rosIMU.linear_acceleration = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    // publish sensors
    imu_publisher_->publish(ros_imu);
 }

void MotionComm::prepareAndPublishFSR()
{
#ifdef VERBOSE
    ROS_INFO("Publishing FSR!");
#endif
    naoqi_bridge_msgs::FloatArrayStamped ros_fsr_l;
    ros_fsr_l.header.stamp = time_;
    ros_fsr_l.header.frame_id = "/l_sole";

    naoqi_bridge_msgs::FloatArrayStamped ros_fsr_r;
    ros_fsr_r.header.stamp = time_;
    ros_fsr_r.header.frame_id = "/r_sole";

    naoqi_bridge_msgs::FloatStamped ros_fsr_l_total;
    ros_fsr_l_total.header.stamp = time_;
    ros_fsr_l_total.header.frame_id = "/l_sole";

    naoqi_bridge_msgs::FloatStamped ros_fsr_r_total;
    ros_fsr_r_total.header.stamp = time_;
    ros_fsr_r_total.header.frame_id = "/r_sole";

    // fill left foot
    ros_fsr_l.data.push_back(fsr_sensor_data_.pressures[Legs::left][FsrSensors::fl]);
    ros_fsr_l.data.push_back(fsr_sensor_data_.pressures[Legs::left][FsrSensors::fr]);
    ros_fsr_l.data.push_back(fsr_sensor_data_.pressures[Legs::left][FsrSensors::bl]);
    ros_fsr_l.data.push_back(fsr_sensor_data_.pressures[Legs::left][FsrSensors::br]);
    ros_fsr_l_total.data = fsr_sensor_data_.totals[Legs::left];

    // fill right foot
    ros_fsr_r.data.push_back(fsr_sensor_data_.pressures[Legs::right][FsrSensors::fl]);
    ros_fsr_r.data.push_back(fsr_sensor_data_.pressures[Legs::right][FsrSensors::fr]);
    ros_fsr_r.data.push_back(fsr_sensor_data_.pressures[Legs::right][FsrSensors::bl]);
    ros_fsr_r.data.push_back(fsr_sensor_data_.pressures[Legs::right][FsrSensors::br]);
    ros_fsr_r_total.data = fsr_sensor_data_.totals[Legs::right];

    // publish
    fsr_l_publisher_->publish(ros_fsr_l);
    fsr_r_publisher_->publish(ros_fsr_r);
    fsr_l_total_publisher_->publish(ros_fsr_l_total);
    fsr_r_total_publisher_->publish(ros_fsr_r_total);

    /*
    geometry_msgs::PointStamped ros_fsr;
    ros_fsr.header.stamp = time_;
    ros_fsr.point.x = fsr_sensor_data_.leftTotal;
    ros_fsr.point.y = fsr_sensor_data_.rightTotal;

    fsr_publisher_->publish(ros_fsr);
    */
}

void MotionComm::prepareAndPublishSystemData()
{
    // TODO
#ifdef VERBOSE
    ROS_INFO("Publishing System Data!");
#endif
    sensor_msgs::BatteryState ros_battery_state;

    ros_battery_state.header.stamp = time_;

    ros_battery_state.current = system_sensor_data_.batteryCurrent;
    ros_battery_state.power_supply_status = system_sensor_data_.batteryCharging? sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING : sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    ros_battery_state.percentage = system_sensor_data_.batteryLevel;

    battery_state_publisher_->publish(ros_battery_state);
}

void MotionComm::prepareAndPublishJointSensorData()
{
    // TODO
#ifdef VERBOSE
    ROS_INFO("Publishing Joint Sensor Data!");
#endif
    naoqi_bridge_msgs::FloatArrayStamped ros_joint_currents;

    ros_joint_currents.data.resize(26);

    // Head angles
    //ros_joints.name[0]      = "HeadYaw";
    ros_joint_currents.data[0]  = joint_sensor_data_.currents[Joints::headYaw];

    //ros_joints.name[1]      = "HeadPitch";
    ros_joint_currents.data[1]  = joint_sensor_data_.currents[Joints::headPitch];

    // Left leg
    //ros_joints.name[2]      = "LHipYawPitch";
    ros_joint_currents.data[2]  = joint_sensor_data_.currents[Joints::lHipYawPitch];

    //ros_joints.name[3]      = "LHipRoll";
    ros_joint_currents.data[3]  = joint_sensor_data_.currents[Joints::lHipRoll];

    //ros_joints.name[4]      = "LHipPitch";
    ros_joint_currents.data[4]  = joint_sensor_data_.currents[Joints::lHipPitch];

    //ros_joints.name[5]      = "LKneePitch";
    ros_joint_currents.data[5]  = joint_sensor_data_.currents[Joints::lKneePitch];

    //ros_joints.name[6]      = "LAnklePitch";
    ros_joint_currents.data[6]  = joint_sensor_data_.currents[Joints::lAnklePitch];

    //ros_joints.name[7]      = "LAnkleRoll";
    ros_joint_currents.data[7]  = joint_sensor_data_.currents[Joints::lAnkleRoll];

    // Right Leg
    //ros_joints.name[8]      = "RHipYawPitch";
    ros_joint_currents.data[8]  = joint_sensor_data_.currents[Joints::rHipYawPitch];

    //ros_joints.name[9]      = "RHipRoll";
    ros_joint_currents.data[9]  = joint_sensor_data_.currents[Joints::rHipRoll];

    //ros_joints.name[10]     = "RHipPitch";
    ros_joint_currents.data[10] = joint_sensor_data_.currents[Joints::rHipPitch];

    //ros_joints.name[11]     = "RKneePitch";
    ros_joint_currents.data[11] = joint_sensor_data_.currents[Joints::rKneePitch];

    //ros_joints.name[12]     = "RAnklePitch";
    ros_joint_currents.data[12] = joint_sensor_data_.currents[Joints::rAnklePitch];

    //ros_joints.name[13]     = "RAnkleRoll";
    ros_joint_currents.data[13] = joint_sensor_data_.currents[Joints::rAnkleRoll];

    // Left arm
    //ros_joints.name[14]     = "LShoulderPitch";
    ros_joint_currents.data[14] = joint_sensor_data_.currents[Joints::lShoulderPitch];

    //ros_joints.name[15]     = "LShoulderRoll";
    ros_joint_currents.data[15] = joint_sensor_data_.currents[Joints::lShoulderRoll];

    //ros_joints.name[16]     = "LElbowYaw";
    ros_joint_currents.data[16] = joint_sensor_data_.currents[Joints::lElbowYaw];

    //ros_joints.name[17]     = "LElbowRoll";
    ros_joint_currents.data[17] = joint_sensor_data_.currents[Joints::lElbowRoll];

    //ros_joints.name[18]     = "LWristYaw";
    ros_joint_currents.data[18] = joint_sensor_data_.currents[Joints::lWristYaw];

    //ros_joints.name[19]     = "LHand";
    ros_joint_currents.data[19] = joint_sensor_data_.currents[Joints::lHand];

    // Right arm
    //ros_joints.name[20]     = "RShoulderPitch";
    ros_joint_currents.data[20] = joint_sensor_data_.currents[Joints::rShoulderPitch];

    //ros_joints.name[21]     = "RShoulderRoll";
    ros_joint_currents.data[21] = joint_sensor_data_.currents[Joints::rShoulderRoll];

    //ros_joints.name[22]     = "RElbowYaw";
    ros_joint_currents.data[22] = joint_sensor_data_.currents[Joints::rElbowYaw];

    //ros_joints.name[23]     = "RElbowRoll";
    ros_joint_currents.data[23] = joint_sensor_data_.currents[Joints::rElbowRoll];

    //ros_joints.name[24]     = "RWristYaw";
    ros_joint_currents.data[24] = joint_sensor_data_.currents[Joints::rWristYaw];

    //ros_joints.name[25]     = "RHand";
    ros_joint_currents.data[25] = joint_sensor_data_.currents[Joints::rHand];;

    // fill header
    ros_joint_currents.header.stamp = time_;
    joints_currents_publisher_->publish(ros_joint_currents);
}

void MotionComm::prepareAndPublishJoints()
{
#ifdef VERBOSE
    ROS_INFO("Publishing Joint States!");
#endif

    sensor_msgs::JointState ros_joints;

    ros_joints.name.resize(26);
    ros_joints.position.resize(26);

    // Head angles
    ros_joints.name[0]      = "HeadYaw";
    ros_joints.position[0]  = joint_angles_.angles[Joints::headYaw];

    ros_joints.name[1]      = "HeadPitch";
    ros_joints.position[1]  = joint_angles_.angles[Joints::headPitch];

    // Left leg
    ros_joints.name[2]      = "LHipYawPitch";
    ros_joints.position[2]  = joint_angles_.angles[Joints::lHipYawPitch];

    ros_joints.name[3]      = "LHipRoll";
    ros_joints.position[3]  = joint_angles_.angles[Joints::lHipRoll];

    ros_joints.name[4]      = "LHipPitch";
    ros_joints.position[4]  = joint_angles_.angles[Joints::lHipPitch];

    ros_joints.name[5]      = "LKneePitch";
    ros_joints.position[5]  = joint_angles_.angles[Joints::lKneePitch];

    ros_joints.name[6]      = "LAnklePitch";
    ros_joints.position[6]  = joint_angles_.angles[Joints::lAnklePitch];

    ros_joints.name[7]      = "LAnkleRoll";
    ros_joints.position[7]  = joint_angles_.angles[Joints::lAnkleRoll];

    // Right Leg
    ros_joints.name[8]      = "RHipYawPitch";
    ros_joints.position[8]  = joint_angles_.angles[Joints::rHipYawPitch];

    ros_joints.name[9]      = "RHipRoll";
    ros_joints.position[9]  = joint_angles_.angles[Joints::rHipRoll];

    ros_joints.name[10]     = "RHipPitch";
    ros_joints.position[10] = joint_angles_.angles[Joints::rHipPitch];

    ros_joints.name[11]     = "RKneePitch";
    ros_joints.position[11] = joint_angles_.angles[Joints::rKneePitch];

    ros_joints.name[12]     = "RAnklePitch";
    ros_joints.position[12] = joint_angles_.angles[Joints::rAnklePitch];

    ros_joints.name[13]     = "RAnkleRoll";
    ros_joints.position[13] = joint_angles_.angles[Joints::rAnkleRoll];

    // Left arm
    ros_joints.name[14]     = "LShoulderPitch";
    ros_joints.position[14] = joint_angles_.angles[Joints::lShoulderPitch];

    ros_joints.name[15]     = "LShoulderRoll";
    ros_joints.position[15] = joint_angles_.angles[Joints::lShoulderRoll];

    ros_joints.name[16]     = "LElbowYaw";
    ros_joints.position[16] = joint_angles_.angles[Joints::lElbowYaw];

    ros_joints.name[17]     = "LElbowRoll";
    ros_joints.position[17] = joint_angles_.angles[Joints::lElbowRoll];

    ros_joints.name[18]     = "LWristYaw";
    ros_joints.position[18] = joint_angles_.angles[Joints::lWristYaw];

    ros_joints.name[19]     = "LHand";
    ros_joints.position[19] = joint_angles_.angles[Joints::lHand];

    // Right arm
    ros_joints.name[20]     = "RShoulderPitch";
    ros_joints.position[20] = joint_angles_.angles[Joints::rShoulderPitch];

    ros_joints.name[21]     = "RShoulderRoll";
    ros_joints.position[21] = joint_angles_.angles[Joints::rShoulderRoll];

    ros_joints.name[22]     = "RElbowYaw";
    ros_joints.position[22] = joint_angles_.angles[Joints::rElbowYaw];

    ros_joints.name[23]     = "RElbowRoll";
    ros_joints.position[23] = joint_angles_.angles[Joints::rElbowRoll];

    ros_joints.name[24]     = "RWristYaw";
    ros_joints.position[24] = joint_angles_.angles[Joints::rWristYaw];

    ros_joints.name[25]     = "RHand";
    ros_joints.position[25] = joint_angles_.angles[Joints::rHand];;

    // fill header
    ros_joints.header.stamp = time_;
    joints_publisher_->publish(ros_joints);
}

} //bhuman2ros
