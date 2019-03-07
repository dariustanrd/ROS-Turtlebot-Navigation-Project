# Turtlebot Autonomous Navigation in ROS Gazebo
<!-- <div align="center">
  <img src="imagelink"><br><br>
</div> -->

| **System Requirements** | **Version / Release** |
|-----------------|-----------|
| Operating System | [Ubuntu 16.04 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/) |
| ROS Distribution Release | [Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)*|
| C++ Version | At least C++11 |


*Note: Ensure ROS Kinetic Kame is installed with the `desktop-full` option. The `ros-kinetic-turtlebot` package is required as well.

## Overview
This project is part of the module EE4308 - Advances in Intelligent Systems and Robotics in the National University of Singapore. The goal of this project is to design an autonomous navigation system for the Turtlebot in the Gazebo environment using ROS, which includes pathfinding as well as obstacle avoidance in a grid like maze world.

For our project, we implemented both the A* search algorithm as our pathfinding tool to plan for the Turtlebot's path to goal depending on the obstacles detected by its on-board RGB-D Kinect sensor. Our system is robust enough to handle both seen and unseen worlds as long as the start and end coordinates remain the same at `(0, 0)` and `(4, 4)` respectively.

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
    $ cp -r worlds docs project_init.sh ../
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
    $ cp -r worlds docs project_init.sh ../
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

1. `pos_info`
   
   This node subscribes to the `odom_combined` topic published by the `robot_pose_ekf` node, which is a refined version of standard odometry provided by `\odom` as it incorporates Extended Kalman Filtering of the odometry data with Inertial Measurement Unit (IMU) data from the Turtlebot. It then publishes the Turtlebot's pose estimate for other nodes to subscribe to.

2. `depth_info`
   
   This node publishes the depth information from the Turtlebot's RGB-D kinect sensor at the middle point, which acts as our basic obstacle detection system for obstacles directly in the path of the Turtlebot.

3. `scan_info`
   
   This node subscribes to the `\scan` topic published by the `laserscan_nodelet_manager` node that obtains the scan depth information from the `depthimage_to_laserscan`, which uses the RGB-D kinect sensor data to create a form of 'fake' laser readings similar to that of a LIDAR sensor. This node then publishes the depth information for the left most, right most and middle readings in the horizontal axis. This data is then used in the `bot_control` node to act as our preemptive obstacle detection system for obstacles that are along the way of the Turtlebot's current intended path.

4. `bot_control`
   
   This node is our main node that conducts the motion control, obstacle avoidance and pathfinding for the Turtlebot. It subscribes to the `/auto_ctrl/scan_info`, `/auto_ctrl/depth_info` and `/auto_ctrl/pos_info` topics and determines the next coordinate to move towards using the algorithms found in the `algo.h` header file while avoiding obstacles, updating the algorithm with presence of obstacles in order to reach the goal coordinates.

### Algorithm file `algo.h`
This file contains 2 algorithms, the Flood Fill as well as A* search algorithms. However, we primarily use the A* search algorithm due to its faster computation time and reliability. This file also has the code for the internal 2D array map stored by the Turtlebot so that the algorithm will be able to compute the next move.

### Launch files

1. `world.launch`
   
   This launch file is launched in the `project_init.sh` shell file to launch the gazebo environment using the defined worlds in `project_init.sh` as well as the Turtlebot base. It also includes the `empty_world.launch` file which sets the different arguments for the launching of gazebo. 
   
   This file can actually be found in the ROS install path `/opt/ros/kinetic/share/turtlebot_gazebo/launch/turtlebot_world.launch` but was included in order to ammend the `empty_world.launch` file path to our version.

2. `empty_world.launch`
   
   This launch file can also be found in the ROS install path `/opt/ros/kinetic/share/gazebo_ros/launch/empty_world.launch`. However, in our version, we added the line `<remap from="tf" to="gazebo_tf"/> ` in order to ensure that the tf tree constructed by Gazebo does not affect the tf tree constructed by Turtlebot. This had to be done in order to use the `robot_pose_ekf` node as there was a conflict in the tf frames created by Gazebo and the `robot_pose_ekf` node which prevented `\odom_combined` from being connected to the tf tree.

3. `bot.launch`
   
   This launch file is our main launch file to run the `robot_pose_ekf` node and the 4 nodes that we wrote mentioned previously.

### Choices of worlds for testing

In order to choose the world that you wish to test the Turtlebot performance in, the `project_init.sh` file needs to be ammended to change the chosen world file.

1. `test_world_1.world`
   
   Time taken to reach goal: ~50s

   This world is the most basic world to test our system as it has the shortest and easiest path in terms of obstacle avoidance to get the Turtlebot from start to goal.
   
   ![World 1](/worlds/World1_sample_images/test-wolrd-1-top-view.jpg)

2. `test_world_2.world`
   
   Time taken to reach goal: ~100s

   This world has the added complexity of more obstacles to be avoided as well as a longer path to travel from start to goal, which increases the chance of the Turtlebot drifting off its intended course.
   
   ![World 2](/worlds/World2_sample_images/world_2_snapshot.jpg)

3. `test_world_trap.world`
   
   Time taken to reach goal: ~2000s

   This world was designed to push our system by added 5 potential dead ends along the path of the Turtlebot and to test if it is able to recover after being trapped in the dead end and navigate to the goal.

   ![World 3](/worlds/World3_sample_images/world_3_top.jpg)