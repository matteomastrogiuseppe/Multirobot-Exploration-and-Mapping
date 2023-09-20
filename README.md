# Multirobot Exploration and Mapping

## Project Overview:

_TBD_

## Installation Requirements

- _ROS Version_ Required: Noetic on Ubuntu 20.04

Installation procedure:

```bash
sudo apt update
sudo sudo apt install ros-noetic-desktop
```

Source the environment:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
- Make sure that _Gazebo_ and _Rviz_ are installed:

```bash
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-rviz
```

## Installation Steps

### TurtleBot Environment

For the first development it is necessary the `TurtleBot` environment.

Follow the ufficial procedure at [Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation) and change _catkin_ws_ with _turtle_ws_

Make sure to add

```bash
export TURTLEBOT3_MODEL=waffle
```

to your _.bashrc_ file

Remember to source this workspace by adding

```bash
source ~/turtle_ws/devel/setup.bash
```

to your _.bashrc_ file


### QR-code Maker

#### 1. Copy the existing Arena

In order to display the correct QR codes in the TurtleBot environment, copy and past the folder

```bash
turtle_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds
turtle_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
turtle_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch
```

into your workspace at the defined positions. In this way you will obtain the QR codes positioned in the TurtleBot Arena.


### RTAB-Map ICP Poit Cloud

To obtain an accurate Point Cloud map of the environment visited the solution by [IntRoLab](https://github.com/introlab) was used. In particular the solution offered by [rtabmap_ros](https://github.com/introlab/rtabmap_ros) allows a seamless integration of the library directly in `ROS`.

To use this solution, run the following bash code to install the dependencies
```bash
sudo apt install ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-navigation ros-noetic-dwa-local-planner
```
or follow the installation instruction at the official page for the [noetic version](https://github.com/introlab/rtabmap_ros/tree/noetic-devel).

In order to use this module run the following bash code after running the code in the previous sections

```bash
roslaunch rtabmap_demos demo_turtlebot3_navigation.launch
```

The information connected to the point cloud will be stored into a database `.db` file. In order to obtain the final Point Cloud run the following module in a bash terminal 

```bash
rtabmap-databaseViewer
```

### Try the installaion

To try the correct installation run

```bash
roslaunch multi_mapping turtlebot3_visp_world.launch
```

and in a different terminal, in order to move he robot

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```