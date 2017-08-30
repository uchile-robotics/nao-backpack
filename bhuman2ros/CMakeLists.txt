cmake_minimum_required(VERSION 2.8.3)
project(bhuman2ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  message_generation
  naoqi_bridge_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## Debug flag
set(CMAKE_BUILD_TYPE Release)

## Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

## Set support for C++11
macro(use_cxx11)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()
endmacro(use_cxx11)

use_cxx11()

## Set source files
list(APPEND SRC_FILES
    src/BHInterface.cpp
    src/BHInterface.h
    src/CommBase.cpp
    src/CommBase.h
    src/CognitionComm.cpp
    src/CognitionComm.h
    src/MotionComm.cpp
    src/MotionComm.h
)

list(APPEND SRC_PLATFORM
    src/Platform/BHAssert.cpp
    src/Platform/BHAssert.h
    src/Platform/File.cpp
    src/Platform/File.h
    src/Platform/Semaphore.h
    src/Platform/SystemCall.cpp
    src/Platform/SystemCall.h
    src/Platform/Thread.h
    src/Platform/Time.cpp
    src/Platform/Time.h
)

list(APPEND SRC_REPRESENTATIONS
    src/Representations/Infrastructure/SensorData/FsrSensorData.h
    src/Representations/Infrastructure/SensorData/InertialSensorData.h
    src/Representations/Infrastructure/SensorData/JointSensorData.h
    src/Representations/Infrastructure/SensorData/KeyStates.h
    src/Representations/Infrastructure/SensorData/SystemSensorData.h
    src/Representations/Infrastructure/SensorData
    src/Representations/Infrastructure/Image.cpp
    src/Representations/Infrastructure/Image.h
    src/Representations/Infrastructure/JointAngles.cpp
    src/Representations/Infrastructure/JointAngles.h

    src/Representations/MotionControl/HeadMotionRequest.h
    src/Representations/MotionControl/KickEngineOutput.h
    src/Representations/MotionControl/KickRequest.cpp
    src/Representations/MotionControl/KickRequest.h
    src/Representations/MotionControl/MotionRequest.h
    src/Representations/MotionControl/MoveToTarget.cpp
    src/Representations/MotionControl/MoveToTarget.h
    src/Representations/MotionControl/SpecialActionRequest.cpp
    src/Representations/MotionControl/SpecialActionRequest.h
    src/Representations/MotionControl/WalkKicks.h
    src/Representations/MotionControl/WalkRequest.h
)

list(APPEND SRC_MODULES
    src/Modules/MotionControl/KickEngine/KickEngineParameters.cpp
    src/Modules/MotionControl/KickEngine/KickEngineParameters.h
)

list(APPEND SRC_TOOLS
    src/Tools/Global.cpp
    src/Tools/Global.h
    src/Tools/Range.h
    src/Tools/SIMD.h

    # Communication
    src/Tools/Communication/UdpComm.cpp
    src/Tools/Communication/UdpComm.h

    # Debugging
    src/Tools/Debugging/DebugDataTable.cpp
    src/Tools/Debugging/DebugDataTable.h
    src/Tools/Debugging/Debugging.h
    src/Tools/Debugging/DebugRequest.cpp
    src/Tools/Debugging/DebugRequest.h
    src/Tools/Debugging/Stopwatch.h
    src/Tools/Debugging/TimingManager.cpp
    src/Tools/Debugging/TimingManager.h

    # Image Processing
    src/Tools/ImageProcessing/ColorModelConversions.h

    # Math
    src/Tools/Math/Angle.h
    src/Tools/Math/Approx.h
    src/Tools/Math/BHMath.h
    src/Tools/Math/Constants.h
    src/Tools/Math/Eigen.h
    src/Tools/Math/EigenArrayExtensions.h
    src/Tools/Math/EigenMatrixBaseExtensions.h
    src/Tools/Math/Pose2f.h
    src/Tools/Math/Pose3f.h
    src/Tools/Math/Random.cpp
    src/Tools/Math/Random.h
    src/Tools/Math/Rotation.h
    src/Tools/Math/RotationMatrix.cpp
    src/Tools/Math/RotationMatrix.h

    # Message Queue
    src/Tools/MessageQueue/InMessage.cpp
    src/Tools/MessageQueue/InMessage.h
    src/Tools/MessageQueue/MessageIDs.h
    src/Tools/MessageQueue/MessageQueue.cpp
    src/Tools/MessageQueue/MessageQueue.h
    src/Tools/MessageQueue/MessageQueueBase.cpp
    src/Tools/MessageQueue/MessageQueueBase.h
    src/Tools/MessageQueue/OutMessage.cpp
    src/Tools/MessageQueue/OutMessage.h

    # Motion
    src/Tools/Motion/SensorData.h

    # Robot Parts
    src/Tools/RobotParts/Arms.h
    src/Tools/RobotParts/FootShape.cpp
    src/Tools/RobotParts/FootShape.h
    src/Tools/RobotParts/FsrSensors.h
    src/Tools/RobotParts/Joints.h
    src/Tools/RobotParts/Legs.h
    src/Tools/RobotParts/Limbs.h

    # Streams
    src/Tools/Streams/AutoStreamable.h
    src/Tools/Streams/Eigen.h
    src/Tools/Streams/Enum.cpp
    src/Tools/Streams/Enum.h
    src/Tools/Streams/EnumIndexedArray.h
    src/Tools/Streams/InOut.cpp
    src/Tools/Streams/InOut.h
    src/Tools/Streams/InStreams.cpp
    src/Tools/Streams/InStreams.h
    src/Tools/Streams/OutStreams.cpp
    src/Tools/Streams/OutStreams.h
    src/Tools/Streams/SimpleMap.cpp
    src/Tools/Streams/SimpleMap.h
    src/Tools/Streams/Streamable.cpp
    src/Tools/Streams/Streamable.h
    src/Tools/Streams/StreamHandler.cpp
    src/Tools/Streams/StreamHandler.h
)

## Catkin package
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  src
)

## Declare a C++ executable
add_executable("${PROJECT_NAME}_node" src/main_node.cpp ${SRC_FILES} ${SRC_MODULES} ${SRC_REPRESENTATIONS} ${SRC_PLATFORM} ${SRC_TOOLS})

## Specify libraries to link a library or executable target against
target_link_libraries("${PROJECT_NAME}_node"
  ${catkin_LIBRARIES}
)

option(WITH_VERBOSE "Enable Verbosity" OFF)
if(WITH_VERBOSE)
  add_definitions(-DVERBOSE)
else()
  remove_definitions(-DVERBOSE)
endif()

message("-- Using Verbosity: " ${WITH_VERBOSE})
