/**
* @file Representations/MotionControl/WalkRequest.h
* This file declares a class that represents a walk request.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
* @author Colin Graf
*/

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

/**
* @class MoveToTarget
* A class that represents a walk request.
*/

STREAMABLE(MoveToTarget,
{
public:
  bool isValid() const,

  (Vector2f)(Vector2f::Zero()) finalTarget,
  (Vector2f)(Vector2f::Zero()) target,
  (Pose2f)(Pose2f()) plannerTarget,
  (float)(0) distance,
  (bool)(false) isLocal,
  (bool)(false) dodgeBall,
  (bool)(false) leftFootSelector,
  (bool)(false) forcedFoot,
 });
