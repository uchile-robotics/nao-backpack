/**
 * @file MotionBackpackComm.cpp
 * This file declares a module that handles UDP communication with an external processing unit
 * @author Matías Mattamala
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/BackpackData.h"

#include "Tools/Communication/UdpComm.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/AutoStreamable.h"
#include <stdio.h>


MODULE(MotionBackpackComm,
{,
  REQUIRES(JointAngles),
  REQUIRES(FsrSensorData),
  REQUIRES(InertialSensorData),
  REQUIRES(JointSensorData),
  REQUIRES(SystemSensorData),
  PROVIDES(MotionBackpackData),
});

class MotionBackpackComm : public MotionBackpackCommBase, public MessageHandler
{
  STREAMABLE(Parameters,
  {,
    (bool)(false) enabled,
    (int)(10030) sensorPort,
    (int)(10040) imagePort,
    (std::string)("192.168.21.100") ip,
  });
  
private:
  const int PACKET_SIZE = 1400;

  // the robot parameters
  Parameters params;

  MessageQueue receiver;  // the receiver queue
  MessageQueue sender;    // the sender queue
  UdpComm socket;         // the UDP socket
  

  // Methods
  void sendMotionPackets();
  void receiveBackpackPackets();
  
  void loadParameters();
  
  bool handleMessage(InMessage& message);
  void update(MotionBackpackData& motionBackpackData);

public:
  // Default constructor
  MotionBackpackComm();
};
