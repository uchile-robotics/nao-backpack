/**
 * @file Representations/MotionControl/MotionRequest.h
 * This file declares a struct that represents the motions that can be requested from the robot.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "SpecialActionRequest.h"
#include "WalkRequest.h"
#include "KickRequest.h"

/**
 * @struct MotionRequest
 * A struct that represents the motions that can be requested from the robot.
 */
STREAMABLE(MotionRequest,
{
  ENUM(Motion,
  {,
    walk,
    kick,
    specialAction,
    stand,
    getUp,
    fall,
  });

  bool isValid() const
  {
    return walkRequest.isValid();
  }

  /** Draws something*/
  void draw() const,

  (Motion)(specialAction) motion, /**< The selected motion. */
  (SpecialActionRequest) specialActionRequest, /**< The special action request, if it is the selected motion. */
  (WalkRequest) walkRequest, /**< The walk request, if it is the selected motion. */
  (KickRequest) kickRequest, /**< The kick request, if it is the selected motion. */
});

class VGMotionRequest : public MotionRequest 
{};

// struct BehaviorMotionRequest : public MotionRequest {};

class BehaviorMotionRequest: public MotionRequest
{
public:
  //BehaviorMotionControl();
  BehaviorMotionRequest(): motionControl(MotionControl::behavior_type) {};

  ENUM(MotionControl,
  {,
    behavior_type,
    dribbling,
    bezier,
    path_planning,
    VGpath_planning,
    goToFuzzy,
    rounding,
    passing,
    inWalkKicking,
  });

  MotionControl motionControl;

  BehaviorMotionRequest& operator=(const MotionRequest& other)
  {
    motion = other.motion;
    specialActionRequest = other.specialActionRequest;
    walkRequest = other.walkRequest;
    kickRequest = other.kickRequest;
    return *this;
  }

  BehaviorMotionRequest& operator=(const BehaviorMotionRequest& other)
  {
    motionControl = other.motionControl;
    motion = other.motion;
    specialActionRequest = other.specialActionRequest;
    walkRequest = other.walkRequest;
    kickRequest = other.kickRequest;
    return *this;
  }

};

