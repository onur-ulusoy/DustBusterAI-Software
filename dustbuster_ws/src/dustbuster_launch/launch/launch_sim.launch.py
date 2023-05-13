#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@brief: Launch file for Dustbuster robot simulation and visualization.

This script runs Gazebo, RViz, Cartographer, Nav2, robot state publisher, joint state publisher, and spawns the robot in the room.

Usage: ros2 launch dustbuster_launch launch_sim.launch.py

@author: Onur Ulusoy
@date: 22.03.2023
@license: MIT
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction

def generate_launch_description():

    # Get the share directory for dustbuster_launch, gazebo_ros
    pkg_dustbuster_launch = get_package_share_directory('dustbuster_launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Get the config directory
    config_dir = os.path.join(pkg_dustbuster_launch, 'config')

    # Declare world argument
    world_file = os.path.join(config_dir, 'gazebo_worlds', 'room.world')
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
            'gui': 'false',
            'headless': 'false',
            'debug': 'false'
        }.items()
    )

    # Robot URDF file path
    robot_urdf_file = os.path.join(pkg_dustbuster_launch, 'description', 'dustbuster.urdf')

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
        arguments=['-topic', 'robot_description', '-entity', 'dustbuster', '-x', '0', '-y', '0', '-z', '0.1225', '-R', '0', '-P', '0', '-Y', '0'],
        output='screen',
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(config_dir, 'rviz', 'dustbuster.rviz')],
        output='screen'
    )

    # Nav2 Bringup launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dustbuster_launch, 'launch', 'nav2_bringup.launch.py')),
        launch_arguments={
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    # Rotate 360 node with a delay of 2 seconds
    rotate_360_node = TimerAction(
        period=5.0,  # delay in seconds
        actions=[
            Node(
                package='dustbuster_navigation',
                executable='rotate360',
                output='screen'
            )
        ]
    )

    # Return the launch description
    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        rviz,
        nav2_bringup_launch,
        rotate_360_node
    ])

