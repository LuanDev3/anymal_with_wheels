# Anymal With Wheels (AWW)
This Project extends the concepts of the Anymal C robot developed by the ANYmal group and uses the simulation of this robot coupled with wheels to realize a hybrid control navigation in environments with obstacles.

## Dependecies
This project depends directly on the Anymal C simple description project developed by ANYmal that can be accessed in this [link](https://github.com/ANYbotics/anymal_c_simple_description). The second dependency is related to the world in Gazebo where the robot makes the trajectory, in order the [clearPathRobotics](https://github.com/clearpathrobotics/cpr_gazebo) is used to generate the world, with the implemented physics.

## Installation

-   Using git (or download the zip file) clone this repository into the "source code" folder of your ROS workspace (e.g. ~/catkin_ws/src ).

```sh
$ cd ~/$YOUR_WORSKPACE/src
$ git clone https://github.com/LuanDev3/anymal_with_wheels.git
```

-   To install dependencies, run:

```sh
$ chmod +x install.sh
./install.sh
```

- To compile the code run:
```sh
$ cd ~/$YOUR_WORSKPACE/src
catkin_make
```
