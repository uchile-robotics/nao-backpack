# B-Human Modules

This folder includes the [B-Human framework](https://github.com/bhuman/BHumanCodeRelease/releases/tag/coderelease2016) compatible files to be run along the **bhuman2ros** package. We provide two modules **CognitionBackpackComm** and **MotionBackpackComm** to be executed in the _Cognition_ and _Motion_ processes respectively, as well as the configuration files required by these modules and the modified `massCalibration.cfg` file to be used with the backpack.



## Required libraries
- [B-Human framework](https://github.com/bhuman/BHumanCodeRelease/releases/tag/coderelease2016)
We assume you are familiar with this framework, please refer their code release for further details.

## Installation
We preserve the same code structure to save the files. It is a bit annoying but avoids giving full paths to explain where to put each file.

**Note 1:** Please not forget to modify the `modules.cfg` file in order to enable the use of our modules. Basically we include the following lines:

    {representation = CognitionBackpackData; provider = CognitionBackpackComm;},
    {representation = MotionBackpackData; provider = MotionBackpackComm;},
    
**Note 2:** If you are interested in running the remote control mode, you must also change the following lines:

    {representation = HeadMotionRequest; provider = CognitionBackpackComm;},
    {representation = MotionRequest; provider = CognitionBackpackComm;},

In addition, you **must** add `HeadMotionRequest`into the `MessageIDs.h` file ($BHDIR/Src/Tools/MessageQueue/MessageIDs.h).

## License
Please see the NAO Backpack package license.

## Troubleshooting
* **The NAO code fails on run time**: If your are experiencing some errors to run the NAO software while using the backpack, please check that your `MessageID.h` file in your B-Human folder is exactly the same as the backpack's. Since this file is basically an enumeration of message identifiers, any difference will cause inconsistencies in message handling between the NAO and the backpack.


## Pending work
* Write an installation script to copy the files into their respective folders

