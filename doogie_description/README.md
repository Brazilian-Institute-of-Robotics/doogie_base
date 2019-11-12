# **doogie_description**

This ROS catkin package provides the ros controllers interface in order to controll **Doogie Mouse** platform.

**Keywords:** Micromouse, IEEE, ROS

**Author: [Caio Amaral]<br />
Affiliation: [BIR - Brazilian Institute of Robotics]<br />
Maintainer: Caio Amaral, caioaamaral@gmail.com**

### Supported Versions

The **doogie_description** package has been tested under [ROS] Kinetic and Ubuntu 16.04 using **Gazebo 7.0.0** (the one that comes with [ROS] Kinetic Desktop-full installation). 

<!-- [![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/) TODO -->

</br>

## Dependencies 
- [robot_state_publisher] (publishes the current joints state as a sensor_msgs/JointState message),
- [urdf] (controller for differential drive wheel systems),
- [xacro] (provides the infrastructure to load, unload, start and stop controllers);

# **Table of Contents**
- [**doogie_description**](#doogiedescription)
    - [Supported Versions](#supported-versions)
  - [Dependencies](#dependencies)
- [**Table of Contents**](#table-of-contents)
- [**Launch files**](#launch-files)
- [**Parameters**](#parameters)

</br>

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
         
          - Default: `$(find doogie_description)/rviz/display.rviz`

</br>

# **Parameters**

- **`robot_description`**

	The name of the robot description parameter.

- **`maze_description`**

    The name of the maze description parameter.


[controller_manager]: https://wiki.ros.org/controller_manager

[diff_drive_controller]: https://wiki.ros.org/diff_drive_controller

[doogie_description]:  https://github.com/Brazilian-Institute-of-Robotics/doogie_description

[doogie_description]:  https://github.com/Brazilian-Institute-of-Robotics/doogie_description

[doogie_simulators]: https://github.com/Brazilian-Institute-of-Robotics/doogie_simulators

[gazebo_ros]: http://wiki.ros.org/gazebo_ros

[gazebo_ros_control]: http://wiki.ros.org/gazebo_ros_control

[gazebo_plugins]: http://wiki.ros.org/gazebo_plugins

[joint_state_controller]: http://docs.ros.org/kinetic/api/joint_state_controller/html/c++/classjoint__state__controller_1_1JointStateController.html

[ros_control / gazebo_ros_control]: https://gazebosim.org/tutorials?tut=ros_control

[ROS]: http://www.ros.org

[URDF]: http://wiki.ros.org/urdf 

[Xacro]: http://wiki.ros.org/xacro 

[Rviz]: http://wiki.ros.org/rviz

[BIR - Brazilian Institute of Robotics]: https://github.com/Brazilian-Institute-of-Robotics

[Caio Amaral]: https://github.com/caioaamaral