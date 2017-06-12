/**
* @file Representations/MotionControl/WalkRequest.cpp
* Implementation of a class that represents the walks that can be requested from the robot.
*/


#include "MoveToTarget.h"

bool MoveToTarget::isValid() const
{
  return !std::isnan(target.x()) && !std::isnan(target.y())
    && !std::isnan(finalTarget.x()) && !std::isnan(finalTarget.y());
}

