# For World 1
# sudo cp ./worlds/test_world_1.world /opt/ros/kinetic/share/turtlebot_gazebo/worlds/
# export TURTLEBOT_GAZEBO_WORLD_FILE="/opt/ros/kinetic/share/turtlebot_gazebo/worlds/test_world_1.world"

# For World 2
sudo cp ./worlds/test_world_2.world /opt/ros/kinetic/share/turtlebot_gazebo/worlds/
export TURTLEBOT_GAZEBO_WORLD_FILE="/opt/ros/kinetic/share/turtlebot_gazebo/worlds/test_world_2.world"

export ROBOT_INITIAL_POSE="-x 0.5 -y 0.5 -Y 1.57"
roslaunch turtlebot_gazebo turtlebot_world.launch

