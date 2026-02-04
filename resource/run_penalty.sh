#!/bin/bash

# Delete old entities
echo "Deleting old entities..."
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'turtlebot3_waffle_pi'}"
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'red_ball'}"

# Wait for Gazebo to process the deletion
sleep 2

# Generate random X and Y coordinates for the robot
RANDOM_X=$(python3 -c "import random; print(round(random.uniform(7.9, 8.1), 2))")
RANDOM_Y=$(python3 -c "import random; print(round(random.uniform(-0.3, 0.3), 2))")
echo "Spawning robot at random coordinates: X=$RANDOM_X, Y=$RANDOM_Y"

# Spawn fresh entities
echo "Spawning fresh entities..."
ros2 run gazebo_ros spawn_entity.py -entity turtlebot3_waffle_pi -file /home/simina/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf -x $RANDOM_X -y $RANDOM_Y -z 0.1 -Y 3.14
ros2 run gazebo_ros spawn_entity.py -entity red_ball -file /home/simina/ros2_ws/src/turtlebot_penalty/models/ball/model.sdf -x 5.0 -y 0.0 -z 0.1

# Run the penalty shooter node
echo "Starting penalty shooter node..."
ros2 run turtlebot_penalty penalty_shooter