# Doogie Perception

## Overview

Perception sub system of Doogie Mouse robot. 

**Keywords:** perception, doogie, micromouse

### License

The source code is released under a [Apache License 2.0](https://github.com/Brazilian-Institute-of-Robotics/jiro_efuse_driver/blob/master/LICENSE).

**Author:** Mateus Menezes<br>
**Affiliation:** Brazilian Institute of Robotics - BIR<br>
**Maintainer:** Mateus Menezes, mateusmenezes95@gmail.com<br>

The doogie_perception package has been tested under ROS Kinetic and Ubuntu 16.04 LTS.

## Installation

### Building from Source:

Attention, if you haven't installed ROS yet,please check [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). Desktop-Full Install is the recommended one in order to work with this repository.

**Building:**

First, lets create a catkin workspace
> Note: Skip this step if your workspace has already created!

```sh
$ mkdir -p ~/doogie_ws/src
```

**doogie_perception** is in doogie_base stack. So let's clone it inside our workspace source and install it.

```sh
$ cd ~/doogie_ws/src
$ git clone https://github.com/Brazilian-Institute-of-Robotics/doogie_base.git
$ cd ..
$ rosdep install --from-paths src --ignore-src -r -y
```

Now, just build your catkin workspace.

```sh
cd ~/doogie_ws
catkin_make
```

Don't forget to source your workspace before using it. 

```sh
source devel/setup.bash
```

### Unit Tests

> TODO

## Usage

Run the main node with
```sh
$ roslaunch doogie_perception doogie_perception.launch
```

On the other hand, to see the maze obstacle matrix data, execute
```sh
$ rostopic echo /maze_obstacle_matrix -c
```

The topic [Nodes](#nodes) show all topics and services available in efuse_driver_node.

## Config file

* **[maze.yaml](config/maze.yaml):** Include parameters for the doogie_perception node

## Launch file

* **[doogie_perception.launch](launch/doogie_perception.launch):** Launch the node of the doogie_perception and load the parameters defined in [maze.yaml](config/maze.yaml)

## Nodes

### doogie_perception_node

Build a maze obstacle matrix from robot position and cell walls data.

#### Published Topics

* **`/maze_obstacle_matrix`** ([doogie_msgs/MazeCellMultiArray])

	Maze obstacle matrix.
  
#### Subscribed Topics

* **`/doogie_position`** ([doogie_msgs/DoogiePosition])

	Robot position in the maze.

* **`/wall_distances`** ([doogie_msgs/WallDistances])

	Distances from the robot to cell walls.

#### Parameters

* **`maze/rows`** (int, default: 16)

	Number of rows of the maze.

* **`maze/columns`** (int, default: 16)

	Number of columns of the maze.

* **`maze/front_distance_threshold`** (float, default: 0.1)

	Threshold value in meters used to infer if there is a wall on the front of the robot.

* **`maze/side_distance_threshold`** (float, default: 0.1)

	Threshold value in meters used to infer if there are walls on the sides of the robot.

## More info

For an overview of the APIs, generate the class documentation by running the following commands:

```sh
$ cd doc/
$ doxygen Doxyfile
```

and the documentation files will be automatically generated.
For more detailed overview of the APIs please refer to the source code (it's fairly simple) and/or to the test cases.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/Brazilian-Institute-of-Robotics/doogie_base/issues).


[ros_driver_base]: (https://github.com/Brazilian-Institute-of-Robotics/ros_driver_base) 
[maze.yaml]: (config/maze.yaml)
[doogie_msgs/MazeCellMultiArray]: https://github.com/Brazilian-Institute-of-Robotics/doogie_base/blob/master/doogie_msgs/msg/MazeCellMultiArray.msg
[doogie_msgs/DoogiePosition]: https://github.com/Brazilian-Institute-of-Robotics/doogie_base/blob/master/doogie_msgs/msg/DoogiePosition.msg
[doogie_msgs/WallDistances]: https://github.com/Brazilian-Institute-of-Robotics/doogie_base/blob/master/doogie_msgs/msg/WallDistances.msg
