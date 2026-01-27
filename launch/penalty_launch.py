import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot_penalty = get_package_share_directory('turtlebot_penalty')
    
    # Set Gazebo model path to include turtlebot_penalty models
    model_path = os.path.join(pkg_turtlebot_penalty, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']
    
    # Gazebo launch
    world = os.path.join(pkg_turtlebot_penalty, 'worlds', 'penalty_field.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world, 'verbose': 'false'}.items()
    )

    # Spawn TurtleBot3 Waffle Pi
    turtlebot_model_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_waffle_pi',
            '-file', turtlebot_model_path,
            '-x', '8.0',
            '-y', '0.0',
            '-z', '0.01',
            '-Y', '3.14'
        ],
        output='screen'
    )

    # Spawn the red ball
    ball_model_path = os.path.join(pkg_turtlebot_penalty, 'models', 'ball', 'model.sdf')
    spawn_ball = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'red_ball',
            '-file', ball_model_path,
            '-x', '5.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Penalty shooter node
    penalty_shooter_node = Node(
        package='turtlebot_penalty',
        executable='penalty_shooter',
        name='penalty_shooter',
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        gazebo,
        spawn_robot,
        spawn_ball,
        penalty_shooter_node
    ])