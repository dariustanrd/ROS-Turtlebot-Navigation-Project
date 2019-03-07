# Turtlebot Autonomous Navigation in ROS Gazebo
<div align="center">
  <img src="imagelink"><br><br>
</div>

## System requirements
| **System** | **Version / Release** |
|-----------------|-----------|
| Operating System | [Ubuntu 16.04 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/) |
| ROS Distribution Release | [Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)*|
| C++ Version | At least C++11 |
*Note: Ensure ROS Kinetic Kame is installed with the `Desktop-Full` option. The `ros-kinetic-turtlebot` package is required as well.

## Setup & Configuration

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

### Start the Gazebo environment:

```shell
$ source devel/setup.bash
$ chmod +x project_init.sh
$ ./project_init.sh
```

### Start automous navigation to goal:
```shell
$ source devel/setup.bash
$ roslaunch bot bot.launch
```
-----------------


## Section 1
### Subsection
*italics*

**bold**

using `inline code`

```shell
$ python
```

[link name](linkhere)

![image name](image.jpg)

## Section 2

## Section 3