# Multirobot Exploration and Mapping

## Project Overview:

_TBD_

## Installation Steps

### _ROS Version_ Required: Noetic on Ubuntu 20.04

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


### Create ROS Workspace
Create the **catkin_ws** workspace:

```bash
mkdir -p ~/turtle_ws/src
```

**Clone** the project repository in the "src" folder:
```bash
cd ~/turtle_ws/src
git clone https://github.com/matteomastrogiuseppe/Multirobot-Exploration-and-Mapping
```

**Build** the workspace and packages:
```bash
cd ~/turtle_ws
catkin_make
```

Remember to **source** this workspace by editing the .bashrc file:
```bash
echo "source ~/turtle_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### TurtleBot Environment

For the first development it is necessary the `TurtleBot` environment.

Either follow the ufficial procedure at [Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation), or simply install via Debian:

```bash
sudo apt-get install ros-noetic-turtlebot3 apt-get install ros-noetic-turtlebot3-gazebo 
```

Make sure to add the Turtlebot model to the .bashrc file:

```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```


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