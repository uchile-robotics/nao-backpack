# Installation instructions

In order to follow the isntructions, we assume that:

* You built your NAO backpack using the same hardware as us, i.e. using the ODROID XU4. Also, we 
* You have already cloned this repository. If not `git clone https://www.github.com/uchile-robotics/nao-backpachk.git`

In addition, we prepared scripts for some steps to install the software dependencies. Remember to check the execution permissions, run `chmod +x the_problematic_script.sh` to enable this.

Also, if you notice that our scripts lack some packages, please **tell us or add them by pull request**.


## Step 1: Ubuntu Installation
For our setup we used Ubuntu 16.04 from the [HardKernel site](http://odroid.com/dokuwiki/doku.php?id=en:xu3_release_linux_ubuntu). Please take care of the [flashing configurations](http://odroid.com/dokuwiki/doku.php?id=en:odroid_flashing_tools)

## Step 2: Odroid utility, Ubuntu dependencies and VNC Configuration

    ./odroid_dependencies.sh

## Step 2: ROS installation
Follow the [Ubuntu ARM Install of ROS Indigo](http://wiki.ros.org/indigo/Installation/UbuntuARM) but **please take care of changing every mention of _indigo_ by _kinetic_**.


## Step 3: Backpack dependencies

    ./backpack_installer.sh

