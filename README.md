# **doogie_base**

The **doogie_base** stack provides [ROS] common packages to use both in simulation and real life application for Doogie Mouse plataform.

**Keywords:** Micromouse, ROS

**Author: [Caio Amaral]<br />
Affiliation: [BIR - Brazilian Institute of Robotics]<br />
Maintainer: Caio Amaral, caioaamaral@gmail.com**

### Supported Versions

- **Kinetic**: Built and tested under [ROS] Kinetic and Ubuntu 16.04

<!-- [![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/) TODO -->

### Dependencies 
- [ROS] : An open-source robot framework.

<br/>

# **Table of Contents**
- [**doogie_base**](#doogiebase)
    - [Supported Versions](#supported-versions)
    - [Dependencies](#dependencies)
- [**Table of Contents**](#table-of-contents)
- [**File System**](#file-system)
- [**Installation**](#installation)
    - [1. Installation from Packages:](#1-installation-from-packages)
    - [2. Building from Source:](#2-building-from-source)
  - [Example of Usage](#example-of-usage)
    - [Visualize Doogie Mouse in Rviz:](#visualize-doogie-mouse-in-rviz)
- [**Purpose of the Project**](#purpose-of-the-project)
- [**License**](#license)
- [**Bugs & Feature Requests**](#bugs--feature-requests)

# **File System**

- [doogie_control] : Define ros_controllers used with Doogie Mouse plataform.
- [doogie_description]: Cotains Doogie Mouse **URDF**.


# **Installation**

### 1. Installation from Packages:

TODO

    sudo apt-get install ros-indigo-...


### 2. Building from Source:

Attention, if you haven't installed [ROS] yet, please check [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). Desktop-Full Install is the recommended one in order to work with this repository.    

**Building:**

First, lets create a catkin workspace.

    mkdir -p ~/catkin_ws/src

Then, clone **doogie_base** inside your workspace source.
        
    git clone http://github.com/doogie-mouse/doogie_base.git

Now, just build your catkin workspace.

    cd ~/catkin_ws
    catkin build

Don't forget to source your workspace before using it.
    
    source devel/setup.bash


## Example of Usage

### Visualize Doogie Mouse in Rviz:

Just launch

	roslaunch doogie_description robot_display.launch

</br>

# **Purpose of the Project**

Doogie Mouse platform was originally developed in 2019 as an undergraduate thesis (Theoprax methodoly) at Centro Universitário SENAI CIMATEC in partnership with Brazillian Institute of Robotics, for teaching ṕrinciples of artificial intelligence using high level framework for writing and reusing robot software.

It's a [open source project](/LICENSE) and expects modifications and contributions from it's users. 

</br>

# **License**

Doogie Base source code is released under a [GPLv3](/LICENSE).

</br>

# **Bugs & Feature Requests**

Please report bugs and request features using the [Issue Tracker].

[BIR - Brazilian Institute of Robotics]: https://github.com/Brazilian-Institute-of-Robotics
[Caio Amaral]: https://github.com/caioaamaral
[doogie_base]: https://github.com/doogie-mouse/doogie_base.git
[doogie_control]: https://github.com/doogie-mouse/doogie_control.git
[doogie_description]: https://github.com/doogie-mouse/doogie_description.git
[doogie_gazebo]: doogie_gazebo
[doogie_gazebo/Tutorials]: https://github.com/doogie-mouse/doogie_base/wiki/doogie_gazebo
[Issue Tracker]: https://github.com/doogie-mouse/doogie_base/issues
[ROS]: https://www.ros.org
