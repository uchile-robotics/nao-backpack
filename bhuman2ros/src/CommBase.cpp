/**
 * @file CommBase.cpp
 * This file implements a template of communication threads between ROS and B-Human
 *
 * This file uses part of the BHuman Code Release 2016 <a href="https://github.com/bhuman/BHumanCodeRelease"</a>)
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */

#include "CommBase.h"

namespace bhuman2ros
{

CommBase::CommBase(std::string name, std::string ip, int port, int buf_size, int rate, boost::mutex& mutex)
    : name_(name), ip_(ip), port_(port), m_buf_size_(buf_size), tmp_buf_size_(2048), rate_(rate)
{
    mutex_ = boost::shared_ptr<boost::mutex>(&mutex);

    // configure UDP
    configureSocket();

    // prepare buffers
    queue_.setSize(m_buf_size_);
    out_queue_.setSize(m_buf_size_);
    m_buf_ = new char [m_buf_size_];
    tmp_buf_ = new char[tmp_buf_size_];
}

CommBase::~CommBase()
{
    if(m_buf_)
        delete [] m_buf_;
    if(tmp_buf_)
        delete [] tmp_buf_;
}

bool CommBase::main()
{
    ros::Rate thread_rate(rate_);
    {
        boost::mutex::scoped_lock lock(*mutex_);
        Global::theStreamHandler = &stream_handler_;
    }

    while(ros::ok())
    {
        ROS_INFO("Main process %s", name_.c_str());
        time_ = ros::Time::now();
        bool sent = sendMessages();
        bool received = receiveMessages();

        if(received)
        {
            ROS_INFO("handling %s messages", name_.c_str());
            queue_.handleAllMessages(*this);
            queue_.clear();
        }
        else
        {
            ROS_INFO("sleeping process %s", name_.c_str());
            thread_rate.sleep();
        }
    }
}

void CommBase::configureSocket()
{
    socket_.setBlocking(false);
    socket_.setBroadcast(false);
    socket_.bind("0.0.0.0", port_);
    socket_.setTarget(ip_.c_str(), port_);
    socket_.setTTL(0);
    socket_.setLoopback(false);
}

} //bhuman2ros
