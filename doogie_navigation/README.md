# **doogie_navigation**

Provides velocity commands to **Doogie Mouse** platform based on odometry, sensor streams and goal pose.

**Keywords:** Micromouse, IEEE, ROS

**Author: [Caio Amaral]<br />
Affiliation: [BIR - Brazilian Institute of Robotics]<br />
Maintainer: Caio Amaral, caioaamaral@gmail.com**

### Supported Versions

The **doogie_navigation** package has been tested under [ROS] Kinetic and Ubuntu 16.04 using **Gazebo 7.0.0** (the one that comes with [ROS] Kinetic Desktop-full installation). 

<!-- [![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/) TODO -->

</br>

## Dependencies 
- [rospy] (It provides a client library that enables Python programmers to quickly interface with ROS Topics, Services, and Parameters),
- [roscpp] (It provides a client library that enables C++ programmers to quickly interface with ROS Topics, Services, and Parameters),
- [doogie_msgs] (provides the ros msgs to interact with **Doogie Mouse** platform);

# **Table of Contents**
- [**doogie_navigation**](#doogienavigation)
    - [Supported Versions](#supported-versions)
  - [Dependencies](#dependencies)
- [**Table of Contents**](#table-of-contents)
    - [1. Core Xacros](#1-core-xacros)
    - [2. Acessories Xacros](#2-acessories-xacros)
    - [3. Arguments](#3-arguments)
- [Mazes Available](#mazes-available)
- [**Launch files**](#launch-files)
- [**Parameters**](#parameters)

</br>

<!-- # **ROS Messages**

[Doogie Mouse URDF](/doogie_navigation/urdf) desing is [xacro] oriented, so it's subdivided in macros submodules designated for each aspect of the robot. They are subdivided in core xacros and acessories xacros:

### 1. Core Xacros

- **common_properties.xacro:** 
   - defines some rviz **colors**;
- **doogie_base.xacro:** 
   - the macro for generating Doogie Mouse **base plat**;
- **doogie_sensors.xacro:**
   - the macro for generating **all sensors** used by Doogie Mouse, each sensor is defined in the xacros files located in [acessories](/doogie_navigation/urdf/acessories) folder.
- **doogie_wheels.xacro:** 
   - the macro for generating **wheels**.

### 2. Acessories Xacros

- **ir_sensor.xacro:** 
   - the macro for generating the **ir sensor** used by Doogie Mouse. 

### 3. Arguments

Doogie Mouse URDF also can be modified by passing it arguments by launchfiles (e.g [robot_description.launch](doogie_navigation/launch/robot_description.launch)) such as:

- **namespace**

   - defines a namespace which will be appended to each link name.
  
- **robotParam**
   
   - defines the parameter name where que robot description will be loaded.

# Mazes Available

   - **Minus** [10x10 Modified IEEE pattern] 

# **Launch files**

1. **robot_description.launch:** load Doogie Mouse robot description parameter.

     - **Arguments to set robot_description**

       - **`robot_namespace:`** specifie the robot URDF namespace.
      
          - Default: `/`
           
       - **`robot_param:`** specifie de parameter name in which will be loaded the robot description.
       
          - Default: `robot_description` 

2. **maze_description:** load some Doogie Mouse's maze description parameter.
   
     - **Arguments to set maze_description**
      
       - **`maze_name:`** set wich maze will be loaded.
         
          - Default: `minus`  

3. **robot_display:** Display Doogie Mouse in Rviz.
   
     - **Arguments to set robot_display**
      
       - **`rvizconfig:`** set wich Rviz setup will be loaded.
         
          - Default: `$(find doogie_navigation)/rviz/display.rviz`

</br>

# **Parameters**

- **`robot_description`**

	The name of the robot description parameter.

- **`maze_description`**

    The name of the maze description parameter. -->


[controller_manager]: https://wiki.ros.org/controller_manager

[diff_drive_controller]: https://wiki.ros.org/diff_drive_controller

[doogie_navigation]:  https://github.com/Brazilian-Institute-of-Robotics/doogie_navigation

[doogie_navigation]:  https://github.com/Brazilian-Institute-of-Robotics/doogie_navigation

[doogie_simulators]: https://github.com/Brazilian-Institute-of-Robotics/doogie_simulators

[gazebo_ros]: http://wiki.ros.org/gazebo_ros

[gazebo_ros_control]: http://wiki.ros.org/gazebo_ros_control

[gazebo_plugins]: http://wiki.ros.org/gazebo_plugins

[joint_state_controller]: http://docs.ros.org/kinetic/api/joint_state_controller/html/c++/classjoint__state__controller_1_1JointStateController.html

[message_generation]: https://wiki.ros.org/message_generation

[message_runtime]: https://wiki.ros.org/message_runtime

[nav_msgs]: https://wiki.ros.org/nav_msgs

[navigation stack]: https://wiki.ros.org/navigation

[ros_control / gazebo_ros_control]: https://gazebosim.org/tutorials?tut=ros_control

[ROS]: https://www.ros.org

[sensor_msgs]: https://wiki.ros.org/sensor_msgs

[std_msgs]: https://wiki.ros.org/std_msgs

[tf]: https://wiki.ros.org/tf

[URDF]: http://wiki.ros.org/urdf 

[Xacro]: http://wiki.ros.org/xacro 

[Rviz]: http://wiki.ros.org/rviz

[robot_state_publisher]: https://wiki.ros.org/robot_state_publisher

[BIR - Brazilian Institute of Robotics]: https://github.com/Brazilian-Institute-of-Robotics

[Caio Amaral]: https://github.com/caioaamaral
