/**
 * @file CommBase.h
 * This file declares a template of communication threads between ROS and B-Human
 *
 * This file uses part of the BHuman Code Release 2016 <a href="https://github.com/bhuman/BHumanCodeRelease"</a>)
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */

#pragma once

// B-Human dependencies
#include "Tools/Communication/UdpComm.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/StreamHandler.h"
#include "Tools/Global.h"

// ROS Dependencies
#include <ros/ros.h>

// Boost
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

// Other C++ dependencies
#include <unistd.h>

namespace bhuman2ros
{

class CommBase : public MessageHandler
{
protected:
    // Private variables
    StreamHandler stream_handler_;
    MessageQueue queue_;
    MessageQueue out_queue_;
    UdpComm socket_;

    std::string name_;

    std::string ip_;        // robot IP
    int port_;              // UDP port
    int rate_;              // Thread rate
    int m_buf_size_;        // size of
    int tmp_buf_size_;      // size of the temporal buffer to wait for packets
    char* m_buf_;           // buffer to save the image (only used in CognitionComm)
    char* tmp_buf_;         // buffer to save the packets
    boost::shared_ptr<boost::mutex> mutex_; // a mutex for the stream handler
    ros::Time time;         // ros time

private:

    void configureSocket();

    // Virtual methods
    virtual bool handleMessage(InMessage &message) = 0;
    virtual bool receiveMessages() = 0;
    virtual bool sendMessages() = 0;

public:
    // Constructor
    CommBase() : ip_(""), port_(0), m_buf_size_(0), rate_(0) {}
    CommBase(std::string name, std::string ip, int port, int buf_size, int rate, boost::mutex &mutex);

    // Destructor
    ~CommBase();

    // Main loop
    bool main();
    bool configureThread(ros::NodeHandle& node_handle);
};

} //bhuman2ros
