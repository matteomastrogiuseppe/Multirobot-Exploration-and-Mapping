# Multirobot Exploration and Mapping

## Project Overview:
To obtain an accurate Point Cloud map of the environment visited the solution by [IntRoLab](https://github.com/introlab) was used. In particular the solution offered by [rtabmap_ros](https://github.com/introlab/rtabmap_ros) allows a seamless integration of the library directly in `ROS`.

The information connected to the point cloud will be stored into a database `.db` file. In order to obtain the final Point Cloud run the following module in a bash terminal 

```bash
rtabmap-databaseViewer
```

_TBD_

## Installation Steps:

### _ROS Version_ and _OS_: Noetic on Ubuntu 20.04

Install ROS Noetic:

```bash
sudo apt update
sudo sudo apt install ros-noetic-desktop
```

Source the ROS environment:

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

For a first development, the `TurtleBot` environment was used.

Follow the ufficial procedure at [Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation), or simply install via Debian:

```bash
sudo apt-get install ros-noetic-turtlebot3 apt-get install ros-noetic-turtlebot3-gazebo 
```

Make sure to add the Turtlebot model to the .bashrc file:

```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

### RTAB-Map and Map Merge:
Run the following bash code to install RTAB-Map:
```bash
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
```

_multirobot_map_merge_ was used to merge the robot individual maps.
```bash
sudo apt install ros-$ROS_DISTRO-multirobot-map-merge
```

### Python 
Install Python 3.8 with the following command:
```bash
sudo apt install python3.8
sudo apt install python3-pip
```
Install the required libraries:
```bash
pip3 install numpy cv2 numba pyfmm
```

## Try the installation

To try the correct installation, run:

```bash
roslaunch multi_explore multi.launch
```
