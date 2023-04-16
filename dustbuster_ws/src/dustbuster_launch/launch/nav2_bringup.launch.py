#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file: Launch file to start nav2 stack

@author: Onur Ulusoy
@date: 01/04/2023

Usage: ros2 launch dustbuster_launch launch_sim.launch.py

@section LICENSE

This code is licensed under the MIT license.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

## @brief Generate the launch description for the navigation_launch file
#
# @return LaunchDescription object containing the navigation launch file
def generate_launch_description():
    # Get the share directory for dustbuster_launch
    pkg_dustbuster_launch = get_package_share_directory('dustbuster_launch')

    # Get the config directory
    config_dir = os.path.join(pkg_dustbuster_launch, 'config')

    nav2_bringup_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Include the navigation launch file
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'navigation_launch.py')),
        launch_arguments={
            'params_file': os.path.join(config_dir, 'nav2_params.yaml'),
            'use_sim_time': 'true',
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(navigation)

    return ld
