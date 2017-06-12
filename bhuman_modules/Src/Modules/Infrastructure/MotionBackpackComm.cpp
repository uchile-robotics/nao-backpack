/**
* @file CognitionBackpackComm.cpp
* Implementation of module CognitionBackpackComm
* @author Matías Mattamala
*/

#include "MotionBackpackComm.h"
#include <stdio.h>
#include <iostream>

MAKE_MODULE(MotionBackpackComm, motionInfrastructure)

MotionBackpackComm::MotionBackpackComm()
{
  // load parameters
  loadParameters();
    
  // initialize message queues
  sender.setSize(2800000);
  receiver.setSize(2800000);
  
  // configure socket
  socket.setBlocking(false);
  socket.setBroadcast(false);
  socket.bind("0.0.0.0", params.motionPort);
  socket.setTarget(params.ip.c_str(), params.motionPort);
  socket.setTTL(0);

  socket.setLoopback(true); // no reception of own packages
}

void MotionBackpackComm::update(MotionBackpackData& MotionBackpackData)
{
  if(params.enabled)
  {
    receiveBackpackPackets();
	sendMotionPackets();
  }
}

void MotionBackpackComm::sendMotionPackets()
{
  // send packets
  // fill sender queue
  sender.out.bin << theInertialSensorData;
  sender.out.finishMessage(idInertialSensorData);
  
  sender.out.bin << theFsrSensorData;
  sender.out.finishMessage(idFsrSensorData);
  
  sender.out.bin << theJointSensorData;
  sender.out.finishMessage(idJointSensorData);
  
  sender.out.bin << theSystemSensorData;
  sender.out.finishMessage(idSystemSensorData);

  sender.out.bin << theJointAngles;
  sender.out.finishMessage(idJointAngles);

  if(sender.getNumberOfMessages())
  {
    int size = sender.getStreamedSize();
	// send a first packet with the image size  

	char* buffer = 0;
	buffer = new char[size];
	OutBinaryMemory stream(buffer);
	stream << sender;
	  
	socket.write(buffer, size);

	delete [] buffer;
  }
  sender.clear();
}

void MotionBackpackComm::receiveBackpackPackets()
{
#ifdef TARGET_ROBOT
  char* buffer = 0;
  buffer = new char[PACKET_SIZE];
  int size;

  size = socket.read(buffer, PACKET_SIZE);
  
  if(size > 0)
  {
    InBinaryMemory memory(buffer, size);
    memory >> receiver;
  }
  
  delete [] buffer;
  
  receiver.handleAllMessages(*this);
  receiver.clear();
#endif
}

bool MotionBackpackComm::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
    {
    default:
        return false;
    }

    return false;
}

void MotionBackpackComm::loadParameters()
{
  InMapFile stream("backpackSettings.cfg");
  if(stream.exists())
  {
    stream >> params;
  }
}