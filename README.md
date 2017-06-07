# The NAO Backpack package

This package accompanies the paper _"The NAO Backpack: An Open-hardware Add-on for Fast Software Development with the NAO Robot"_ accepted in the [RoboCup Symposium 2017](https://www.robocup2017.org/).

**To cite this work:**

Matías Mattamala, Gonzalo Olave, Clayder González, Nicolás Hasbún, and Javier Ruiz-del-Solar. The NAO Backpack: An Open-hardware Add-on for Fast Software Development with the NAO Robot. RoboCup Symposium 2017, NAgoya, Japan, 2017. **TODO** [PDF](https://github.com/uchile-robotics/nao-backpack)

**Bibtex**

    @article{mattamalaRoboCup2017,
      title={ {The NAO Backpack: An Open-hardware Add-on for Fast Software Development with the NAO Robot}},
      author={Mattamala, Mat\'ias, Olave, Gonzalo, Gonz\'alez, Clayder, Hasb\'un, Nicol\'as, and Ruiz-del-Solar, Javier},
      year = {2017},
    }
    
This package includes:

- A CAD model to print yout own NAO backpack
- An URDF model with the backpack
- Some [B-Human framework](https://github.com/bhuman/BHumanCodeRelease)'s modules to publish internal representations via UDP
- A ROS node (**bhuman2ros**) to publish the UDP messages from the NAO to ROS topics
## Overview
### CAD model

### URDF model

### B-Human's framework related modules

### bhuman2ros

## License
This work is licensed under ** define**

## Dependencies
This package is built upon other libraries and frameworks. Here we list them, their respective authors and licenses.

* The **B-Human modules** provided here were developed to be compatible with the [B-Human Framework](https://github.com/bhuman/BHumanCodeRelease), as well as the _representations_ and _communication libraries_ used in the **bhuman2ros** node. The framework includes its own license; details can be found [here](https://github.com/bhuman/BHumanCodeRelease/blob/master/License.txt)
* The **URDF model we provide here** is based on [Armin Hornung's](http://wiki.ros.org/nao_robot), licensed under the BSD.





