#!/bin/bash
colcon build --packages-select turtlebot_penalty
source /home/simina/ros2_ws/install/setup.bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/simina/ros2_ws/src/turtlebot_penalty/models
ros2 launch turtlebot_penalty penalty_launch.py
