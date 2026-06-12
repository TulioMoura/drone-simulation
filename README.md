# Drone Simulation Project
A simulation environment for multiple drones using the simulator _Webots_ and the framework _ROS2 (Robot Operating System).

## Table of Contents
- [Installation](#installation)
- [Usage]($usage)
- [Architecture](#architecture)

<h2 id="installation">Installation</h2>
This guide was made targeting Ubuntu, version 24.04
This project requires the instalation of ROS2, Webots-ros-driver, colcon and Webots to work.
The recommended version is ROS2 Kilted, which can be installed by following the instructions on ROS2 Kilted official website:

Step 1: Install ROS2: 
[ROS2 Humble Installation Guide](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)

Step 2: Install Webots-ros-driver: 
[Webots-Ros-Driver Installation Guide](https://docs.ros.org/en/kilted/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html)

Step 3: Install colcon
```bash
sudo apt update && sudo apt install colcon
```

Step 4: Install Webots:
```bash
https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb &&
sudo apt install ./webots_2025a_amd64.deb
```

Step 5: Clone This repository.
<h2 id="usage">Usage</h2>

### Activate environment
In order to run the simulation, first you need to activate ROS2 on your machine by running the following command inside the project folder

```bash
source /opt/ros/kilted/setup.bash
```

Then, make sure you're inside the project workspace and run
```bash
cd drone-simulation/src/mavic_simulation
source install/local_setup.bash
```

### Run simulation
Before running the simulation, run the following command, which will build ROS2 nodes 
```bash
colcon build
```

Then, in order to run the simulation, run the following command
```bash
ros2 launch mavic_simulation robot_launch.py file:=path5.json
```
This will execute the launcher of the project, install Webots if you don't have it already installed, and open the Webots window of the simulaton.

The "file" parameter on the end of the command, defines the initialization file used on the simulation, the file needs to be located inside the /path directory, and, after any changes on this directory, the project needs to be recompiled using "colcon build" to ensure that the project will load the correct initialization file. 

This repository contais sample files under the /path directory, that can be used to help with the creation of custom initialization files.



