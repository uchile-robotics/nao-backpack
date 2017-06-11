/**
* @file CognitionBackpackComm.cpp
* Implementation of module CognitionBackpackComm
* @author Matías Mattamala
*/

#include "CognitionBackpackComm.h"
#include "Platform/SystemCall.h"

#include <stdio.h>
#include <iostream>

MAKE_MODULE(CognitionBackpackComm, cognitionInfrastructure)

CognitionBackpackComm::CognitionBackpackComm()
{
  // load parameters
  loadParameters();
  
#ifndef TARGET_ROBOT
  params.ip = "127.0.0.1";
#endif
  
  // initialize message queues
  sender.setSize(2800000);
  receiver.setSize(2800000);
  
  // configure image socket 
  imageSocket.setBlocking(false);
  imageSocket.setBroadcast(false);
  imageSocket.bind("0.0.0.0", params.imagePort);
  imageSocket.setTarget(params.ip.c_str(), params.imagePort);
  imageSocket.setTTL(0);
  imageSocket.setLoopback(false); // no reception of own packages
}

void CognitionBackpackComm::update(CognitionBackpackData& cognitionBackpackData)
{
  if(params.enabled)
  {
	receiveBackpackPackets();
    sendImagePackets();
  }
}

void CognitionBackpackComm::update(MotionRequest& motionRequest)
{
  motionRequest = this->motionRequest;
}

void CognitionBackpackComm::update(HeadMotionRequest& headMotionRequest)
{
  headMotionRequest = this->headMotionRequest;
}

void CognitionBackpackComm::sendImagePackets()
{
  CameraInfo::Camera selectedCamera = params.sendUpper ? CameraInfo::upper : CameraInfo::lower;
  if(theCameraInfo.camera == selectedCamera)
  {
    sender.out.bin << theImage; 
    sender.out.finishMessage(idImage);
    
    if(sender.getNumberOfMessages())
    {
      unsigned int imageSize = sender.getStreamedSize();
      
      // send a first packet with the image size  
      imageSocket.write( (char*)&imageSize, sizeof(unsigned int));
      
      // send the image
      char* buffer = 0;
      buffer = new char[imageSize];
      OutBinaryMemory stream(buffer);
      stream << sender;
      
      unsigned int i = 0;
      for(i=0; i<(imageSize-PACKET_SIZE); i+=PACKET_SIZE)
      {
        imageSocket.write(&buffer[i], PACKET_SIZE);
      }
      imageSocket.write(&buffer[i], (imageSize-i) );
      
      // clear buffer
      delete [] buffer;
    }
  }
  sender.clear();
}

void CognitionBackpackComm::receiveBackpackPackets()
{
#ifdef TARGET_ROBOT
  char* buffer = 0;
  buffer = new char[PACKET_SIZE];
  int size;
  
  do{
	size = imageSocket.read(buffer, PACKET_SIZE);
	
    if(size > 0)
    {
	  InBinaryMemory memory(buffer, size);
	  memory >> receiver;
	  break;
    }
  }while(size > 0);

  delete [] buffer;
  
  receiver.handleAllMessages(*this);
  receiver.clear();
#endif
}

bool CognitionBackpackComm::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
    {
	case idMotionRequest:
    {
        message.bin >> motionRequest;
        return true;
    }
	case idHeadMotionRequest:
    {
        message.bin >> headMotionRequest;
        return true;
    }
    default:
        return false;
    }

    return false;
}

void CognitionBackpackComm::loadParameters()
{
  InMapFile stream("backpackSettings.cfg");
  if(stream.exists())
  {
    stream >> params;
  }
}