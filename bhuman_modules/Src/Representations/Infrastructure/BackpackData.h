/**
 * @file BackpackData.h
 * Declaration of a class representing the data sent to the NAO backpack
 * @author Matías Mattamala
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include <stdint.h>


/**
 * @class BackpackData
 * A class representing information received from the NAO backpack
 */
STREAMABLE(BackpackDataReceiver,
{,
  (unsigned)(0) currentTimestamp,
});

/**
 * @class CognitionBackpackDataSenderOutput
 * An empty dummy representation for the CognitionBackpackDataSender module
 */
STREAMABLE(CognitionBackpackData,
{,
});

/**
 * @class MotionBackpackDataSenderOutput
 * An empty dummy representation for the MotionBackpackDataSender module
 */
STREAMABLE(MotionBackpackData,
{,
});