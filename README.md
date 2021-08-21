# Anymal With Wheels (AWW)
This Project extends the concepts of the Anymal C robot developed by the ANYmal group and uses the simulation of this robot coupled with wheels to realize a hybrid control navigation in environments with obstacles.

## Dependecies
This project depends directly on the Anymal C simple description project developed by ANYmal that can be accessed in this [link](https://github.com/ANYbotics/anymal_c_simple_description). The second dependency is related to the world in Gazebo where the robot makes the trajectory, in order the [clearPathRobotics](https://github.com/clearpathrobotics/cpr_gazebo) is used to generate the world, with the implemented physics.

## Installation

-   Using git (or download the zip file) clone this repository into the "source code" folder of your ROS workspace (e.g. ~/catkin_ws/src ).

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/LuanDev3/anymal_with_wheels.git
```

-   Fixing package dependencies:

```sh
$ cd ~/catkin_ws
$ rosstack profile && rospack profile
$ rosdep install --from-paths src/anymal_with_wheels --ignore-src -r -y
```

-   Compile your ROS workspace directory (e.g. ~/catkin-ws ):

```sh
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash # Set the appropriate bash extension
```
