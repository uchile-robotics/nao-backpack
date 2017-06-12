#include "JointAngles.h"
//#include "Tools/Debugging/DebugDrawings.h"

Angle JointAngles::mirror(Joints::Joint joint) const
{
  switch(joint)
  {
    case Joints::headYaw:
      return mirror(angles[Joints::headYaw]);
    case Joints::lShoulderPitch:
    case Joints::lHand:
      return angles[joint - Joints::lShoulderPitch + Joints::rShoulderPitch];
    case Joints::lElbowRoll:
    case Joints::lShoulderRoll:
    case Joints::lElbowYaw:
    case Joints::lWristYaw:
      return mirror(angles[joint - Joints::lShoulderPitch + Joints::rShoulderPitch]);
    case Joints::rShoulderPitch:
    case Joints::rHand:
      return angles[joint - Joints::rShoulderPitch + Joints::lShoulderPitch];
    case Joints::rElbowRoll:
    case Joints::rShoulderRoll:
    case Joints::rElbowYaw:
    case Joints::rWristYaw:
      return mirror(angles[joint - Joints::rShoulderPitch + Joints::lShoulderPitch]);
    case Joints::lHipYawPitch:
    case Joints::lHipPitch:
    case Joints::lKneePitch:
    case Joints::lAnklePitch:
      return angles[joint - Joints::lHipYawPitch + Joints::rHipYawPitch];
    case Joints::lHipRoll:
    case Joints::lAnkleRoll:
      return mirror(angles[joint - Joints::lHipYawPitch + Joints::rHipYawPitch]);
    case Joints::rHipYawPitch:
    case Joints::rHipPitch:
    case Joints::rKneePitch:
    case Joints::rAnklePitch:
      return angles[joint - Joints::rHipYawPitch + Joints::lHipYawPitch];
    case Joints::rHipRoll:
    case Joints::rAnkleRoll:
      return mirror(angles[joint - Joints::rHipYawPitch + Joints::lHipYawPitch]);
    default:
      return angles[joint];
  }
}
