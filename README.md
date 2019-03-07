# Turtlebot Autonomous Navigation in ROS Gazebo
<!-- <div align="center">
  <img src="imagelink"><br><br>
</div> -->

### System requirements
| **System** | **Version / Release** |
|-----------------|-----------|
| Operating System | [Ubuntu 16.04 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/) |
| ROS Distribution Release | [Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)*|
| C++ Version | At least C++11 |


*Note: Ensure ROS Kinetic Kame is installed with the `Desktop-Full` option. The `ros-kinetic-turtlebot` package is required as well.

## Overview
This project is ...

## Setup & configuration

### Setting up this repository with a **new** catkin workspace

1. Create a new catkin workspace.
   
    In your terminal shell, navigate to your intended directory for your new workspace and execute the following commands, replacing `<wsname>` with your intended workspace name.

    ```shell
    $ mkdir -p <wsname>/src
    $ cd <wsname>/src/
    $ catkin_init_workspace
    $ cd ..
    $ catkin_make
    ```

2. Clone this repository and copy necessary folders into your catkin workspace.
    
    In the same `<wsname>` directory, execute the following:

    ```shell
    $ git clone https://github.com/dariustanrd/ROS-Turtlebot-Navigation-Project.git
    $ cd ROS-Turtlebot-Navigation-Project/
    $ cp -r src/bot ../src
    $ cp -r visualisations worlds docs project_init.sh ../
    $ cd ..
    $ catkin_make
    ```

### Setting up this repository with an **existing** catkin workspace
1. Clone this repository and copy necessary folders into your existing catkin workspace.
    
    In your terminal shell, navigate to your existing catkin workspace root directory and execute the following:

    ```shell
    $ git clone https://github.com/dariustanrd/ROS-Turtlebot-Navigation-Project.git
    $ cd ROS-Turtlebot-Navigation-Project/
    $ cp -r src/bot ../src
    $ cp -r visualisations worlds docs project_init.sh ../
    $ cd ..
    $ catkin_make
    ```

## Running the code

### Starting the Gazebo environment:

To initialise the Gazebo world, run the following commands in a terminal:

```shell
$ source devel/setup.bash
$ chmod +x project_init.sh
$ ./project_init.sh
```

### Starting autonomous navigation to goal:
In a new terminal, run the following commands to begin the Turtlebot autonomous navigation.

```shell
$ source devel/setup.bash
$ roslaunch bot bot.launch
```
When the Turtlebot has reached the goal coordinates, the notification will be printed out on the terminal together with the time taken to reach the goal.

-----------------

## Code analysis & explanations

### Nodes
There are 4 main nodes that we wrote for this package which handles different aspects of the autonomous navigation of the Turtlebot.

1. `pos_info.cpp`
   
   This node

2. `depth_info.cpp`
   
   This node

3. `scan_info.cpp`
   
   This node

4. `bot_control.cpp`
   
   This node

### Algorithm file `algo.h`
This file contains 2 algorithms, Flood Fill as well as A*.

### Launch files

### World selection
