# Multirobot Exploration and Mapping

## Project Overview:

_TBD_

## Installation Requirements

- _ROS Version_ Required: Noetic on Ubuntu 20.04

Follow the ufficial procedure at [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

- Make sure that _Gazebo_ and _Rviz_ are installed:

```bash
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

```bash
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

### Vision Visp

For the detection of the various QR codes it is necessary the package [ViSP stack for ROS](https://github.com/lagadic/vision_visp/tree/noetic)

Follow the ufficial procedure at [vision_visp](http://wiki.ros.org/vision_visp) (probably it will be neccesary to install from source, remember that the workspace is called turtle_ws)

In order to try the correct installation, run

```bash
roslaunch visp_auto_tracker tutorial.launch
```

### QR-code Maker

#### 1. Copy the existing Arena

In order to display the correct QR codes in the TurtleBot environment, copy and past the folder

```bash
turtle_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds
turtle_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
turtle_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch
```

into your workspace at the defined positions. In this way you will obtain the QR codes positioned in the TurtleBot Arena.

#### 2. Create QR codes form scratch

For this project the repo [gazebo_models](https://github.com/mikaelarguedas/gazebo_models) was used for the creation of the models.
In order to create the proper QR-codes is necessary to change the code since for `Vision Visp` it is necessary to print a black square around the QR-code requested. In order to do this it is necessary to change the code at line 93 with 

```python
if white_contour_px > 0:
        convert_cmd = "convert %s -bordercolor white -border %dx%d %s" % (
            image_dest_path, 57,
            57, image_dest_path)
        if args.verbose:
            print(convert_cmd)
        os.system(convert_cmd)
        convert_cmd = "convert %s -bordercolor black -border %dx%d %s" % (
            image_dest_path, 150,
            150, image_dest_path)
        if args.verbose:
            print(convert_cmd)
        os.system(convert_cmd)
        convert_cmd = "convert %s -bordercolor white -border %dx%d %s" % (
            image_dest_path, 10,
            10, image_dest_path)
        if args.verbose:
            print(convert_cmd)
        os.system(convert_cmd)
```
This will set the border as required. Be aware that changes might be required, depending on the dimension of the original QR-code.

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