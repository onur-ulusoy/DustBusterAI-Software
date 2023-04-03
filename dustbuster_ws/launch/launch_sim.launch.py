"""
launch_sim.launch.py

Standalone launch file to run Gazebo, RViz, robot state publisher, joint state publisher, and spawn the robot in the room.

Usage: ros2 launch launch_sim.launch.py

Author: Onur Ulusoy
Date: 22.03.2023

This code is licensed under the MIT license.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    # Get the current working directory
    cwd = os.getcwd()

    # Get the share directory for gazebo_ros
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Declare world argument
    world_file = os.path.join(cwd, 'room.world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to world file to load'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'false',
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items()
    )

    # Robot URDF file path
    robot_urdf_file = cwd + '/description/dustbuster.urdf'

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=[robot_urdf_file],
        parameters=[{'use_sim_time': True}],
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot_draft'],
        output='screen',
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', 'room.rviz'],
        output='screen'
    )

    # Get the config directory
    config_dir = cwd + '/config'

    # Slam launch
    online_async_launch_file = os.path.join(cwd, 'online_async_launch.py')
    online_async_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(online_async_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(config_dir, 'mapper_params_online_async.yaml'),
        }.items()
    )

    # Return the launch description
    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        rviz,
        online_async_launch
    ])
