# **doogie_control**

This ROS catkin package provides the ros controllers interface in order to controll **Doogie Mouse** platform.

**Keywords:** Micromouse, IEEE, ROS

**Author: [Caio Amaral]<br />
Affiliation: [BIR - Brazilian Institute of Robotics]<br />
Maintainer: Caio Amaral, caioaamaral@gmail.com**

### Supported Versions

The **doogie_control** package has been tested under [ROS] Kinetic and Ubuntu 16.04 using **Gazebo 7.0.0** (the one that comes with [ROS] Kinetic Desktop-full installation). 

<!-- [![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/) TODO -->

<p align="center">
   <img src="docs/doogie_control.png" alt="doogie_control sim" title="Example Image">
</p>
</br>

## Dependencies 
- [joint_state_controller] (publishes the current joint state as a sensor_msgs/JointState message),
- [diff_drive_controller] (controller for differential drive wheel systems)
- [controller_manager] (provides the infrastructure to load, unload, start and stop controllers), 
- [doogie_description] (package with doogie URDF),

# **Table of Contents**
- [**doogie_control**](#doogiecontrol)
    - [Supported Versions](#supported-versions)
  - [Dependencies](#dependencies)
- [**Table of Contents**](#table-of-contents)
- [**Launch files**](#launch-files)
- [**Controllers Defined**](#controllers-defined)

</br>

# **Launch files**

1. **robot_control.launch:** load and spawn Doogie Mouse controllers, needs simulation or robot_application lauch before using it.

     - **Arguments to set Doogie Mouse spawn**

       - **`ns:`** specifie the namespace in which will be spawned the controllers.
      
          - Default: `/`

</br>

# **Controllers Defined**

**doogie_control** uses controls provided by **ros_controllers** package, as they are the default controllers used by ROS. The **ros_controllers** used in this simulation are:

   - **move_base_controller** ([diff_drive_controller]/DiffDriveController): controls Doogie Mouse wheels.
   - **joint_publisher** ([joint_state_controller]/JointStateController): provides wheel's joints state.

[diff_drive_controller]: https://wiki.ros.org/diff_drive_controller

[doogie_description]:  https://github.com/Brazilian-Institute-of-Robotics/doogie_description

[doogie_control]:  https://github.com/Brazilian-Institute-of-Robotics/doogie_control

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