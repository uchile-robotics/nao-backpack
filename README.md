# The NAO Backpack package

<img src="backpack_cad/img/real_nao_with_backpack.JPG"  width="300" align="right"/>

This package accompanies the paper _"The NAO Backpack: An Open-hardware Add-on for Fast Software Development with the NAO Robot"_ accepted in the [RoboCup Symposium 2017](https://www.robocup2017.org/). Feel free to contribute and collaborate via **issues** or **pull requests** :)

This package includes:

- A [**CAD model**](/backpack_cad) and instructions to build your own NAO backpack.
- An **URDF model** with the backpack. (**pending**)
- Some [**B-Human framework's modules**](/bhuman_modules) to publish internal representations via UDP as well as **configuration files**.
- A [**ROS node (bhuman2ros)**](/bhuman2ros) to publish the UDP messages from the NAO to ROS topics.
- [**Installation**](/install_scripts) and [**configuration**](/config_scripts) scripts to facilitate backpack's software installation and connection to the NAO.

## Instructions

* For an overview of this project, please refer the [**preprint**](https://arxiv.org/abs/1706.06696).
* Installation of the [**backpack software**](https://github.com/uchile-robotics/nao-backpack/tree/master/install_scripts).
* Installation of the [**NAO software**](https://github.com/uchile-robotics/nao-backpack/tree/master/bhuman_modules).
* How to [**configurate and run everything**](https://github.com/uchile-robotics/nao-backpack/tree/master/config_scripts).

## To cite this work

Matías Mattamala, Gonzalo Olave, Clayder González, Nicolás Hasbún, and Javier Ruiz-del-Solar. The NAO Backpack: An Open-hardware Add-on for Fast Software Development with the NAO Robot. RoboCup Symposium 2017, Nagoya, Japan, 2017.

**Bibtex**

    @article{mattamalaRoboCup2017,
      title={ {The NAO Backpack: An Open-hardware Add-on for Fast Software Development with the NAO Robot}},
      author={Mattamala, Mat\'ias, Olave, Gonzalo, Gonz\'alez, Clayder, Hasb\'un, Nicol\'as, and Ruiz-del-Solar, Javier},
      year = {2017},
    }

## License
<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.

This package is built upon other libraries and frameworks:

* The **CAD model** is partially based on [Softbank's](https://www.ald.softbankrobotics.com/en) Solidwork files.
* The **B-Human modules** provided here were developed to be compatible with the [B-Human Framework](https://github.com/bhuman/BHumanCodeRelease), as well as the _representations_ and _communication libraries_ used in the **bhuman2ros** node. The framework includes its own license; details can be found [here](https://github.com/bhuman/BHumanCodeRelease/blob/master/License.txt).
* The **URDF model** is based on [Armin Hornung's](http://wiki.ros.org/nao_robot), licensed under the [BSD](https://github.com/ros-naoqi/nao_robot/blob/master/LICENSE.txt).
* The `set_ros_master.sh` script is based on [Duckietown's](http://wwww.github.com/duckietown/Software.git).
* The `hotspot.sh` script is based on [Duckietown Chile's](http://www.github.com/Duckietown-Chile/Software.git).
