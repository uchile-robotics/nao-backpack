/**
 * @file CognitionBackpackComm.cpp
 * This file declares a module that handles UDP communication with an external processing unit
 * @author Matías Mattamala
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/BackpackData.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"

#include "Tools/Communication/UdpComm.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/StreamHandler.h"
#include <stdio.h>


MODULE(CognitionBackpackComm,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(Image),
  REQUIRES(RobotDimensions),
  PROVIDES(CognitionBackpackData),
  PROVIDES(MotionRequest),
  PROVIDES(HeadMotionRequest),
});

class CognitionBackpackComm : public CognitionBackpackCommBase, public MessageHandler
{
  STREAMABLE(Parameters,
  {,
    (bool)(false) enabled,
	(bool)(true) sendUpper,       // If we want to send the upper or lower image
    (int)(10030) motionPort,
    (int)(10040) cognitionPort,
    (std::string)("192.168.21.100") ip,
  });
    
private:
  const int PACKET_SIZE = 1400;

  Parameters params;
   
    // external packets
  MotionRequest motionRequest;
  HeadMotionRequest headMotionRequest;

  MessageQueue receiver;    // the receiver queue
  MessageQueue sender;      // the sender queue
  UdpComm cognitionSocket;  // A socket to send the cognition info except the image
  UdpComm imageSocket;      // A socket to send the image only

  void loadParameters();
  void sendCognitionPackets();
  void sendImagePackets();
  void receiveBackpackPackets();
  
  bool handleMessage(InMessage& message);
  void update(CognitionBackpackData& cognitionBackpackData);
  
  void update(MotionRequest& motionRequest);
  void update(HeadMotionRequest& headMotionRequest);

public:
  // Default constructor
  CognitionBackpackComm();
};
