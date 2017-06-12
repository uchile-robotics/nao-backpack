/**
 * @file Stopwatch.h
 * The file declares the stopwatch macros.
 * @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:arneboe@tzi.de">Arne Böckmann</a>
 * @author <a href="mailto:fthielke@tzi.de">Felix Thielke</a>
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "TimingManager.h"
#include "Debugging.h"
#include "Tools/Module/Blackboard.h"

/**
 * Helper function to declare a plot as a single statement.
 * @param id The id of the plot.
 * @param value The time to plot in ms.
 */
inline void _plot(const char* id, unsigned time)
{
  DEBUG_RESPONSE(id) OUTPUT(idPlot, bin, (id + 5) << static_cast<float>(time) * 0.001f);
}

#ifndef TARGET_TOOL

/**
 * Allows the measurement the execution time of the following block.
 * @param eventID The id of the stop watch.
 */
#define STOPWATCH(eventID) \
  for(bool _start = true; (_start ? Global::getTimingManager().startTiming(eventID) : (void) Global::getTimingManager().stopTiming(eventID)), _start; _start ^= true)

/**
 * Allows the measurement the execution time of the following block and plot the measurements.
 * @param eventID The id of the stop watch.
 */
#define STOPWATCH_WITH_PLOT(eventID) \
  for(bool _start = true; (_start ? Global::getTimingManager().startTiming(eventID) : _plot("plot:stopwatch:" eventID, Global::getTimingManager().stopTiming(eventID))), _start; _start ^= true)



/*
 * Starts measuring time in a module PER CAMERA.
 * @param moduleName. Module to profile, needs to require CameraInfo
 */
#define CAMERA_TIMING_START(moduleName) \
  TimingManager& _tm = Global::getTimingManager(); \
  if(theCameraInfo.camera == CameraInfo::lower) \
    _tm.startTiming(#moduleName"Lower"); \
  else \
    _tm.startTiming(#moduleName"Upper");

/*
 * Stops measuring time in a module PER CAMERA.
 * @param moduleName. Module to profile, needs to require CameraInfo
 */
#define CAMERA_TIMING_STOP(moduleName) \
  if(theCameraInfo.camera == CameraInfo::lower) \
    _tm.stopTiming(#moduleName"Lower"); \
  else \
    _tm.stopTiming(#moduleName"Upper");

#define TIMING_START(moduleName) \
  TimingManager& _tm = Global::getTimingManager(); \
  _tm.startTiming(moduleName);

#define TIMING_STOP(moduleName) \
  _tm.stopTiming(moduleName);

#else

#define CAMERA_TIMING_START(moduleName) ((void) 0)
#define CAMERA_TIMING_STOP(moduleName) ((void) 0)
#define TIMING_START(moduleName) ((void) 0)
#define TIMING_STOP(moduleName) ((void) 0)
#define STOPWATCH(eventID)
#define STOPWATCH_WITH_PLOT(eventID)

#endif
