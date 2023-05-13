#!/usr/bin/env python3
"""
@file: Dustbuster Launch Package Setup
@author: Onur Ulusoy
@date: 2023-04-16
@license: MIT

This script sets up the Dustbuster Launch package. It handles the inclusion
of required directories and files, as well as package metadata and entry points.
"""

import os
from glob import glob
from setuptools import setup
from pathlib import Path

package_name = 'dustbuster_launch'

def collect_files(directory):
    collected_files = []
    for path in Path(os.path.join(package_name, directory)).rglob('*.*'):
        install_path = os.path.join('share', package_name, str(path.parent)[len(package_name) + 1:])
        collected_files.append((install_path, [str(path)]))
    return collected_files

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        # Include all files in the config directory, including subdirectories
        *collect_files('config'),
        # Include all files in the description directory, including subdirectories
        *collect_files('description'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Onur Ulusoy',
    maintainer_email='onurulusoys4@gmail.com',
    description='Launch files for the Dustbuster robot simulation and navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': [
            'launch_sim = dustbuster_launch.launch_sim:generate_launch_description',
            'nav2_bringup = dustbuster_launch.nav2_bringup:generate_launch_description',
            'online_async_launch = dustbuster_launch.online_async_launch:generate_launch_description',
        ],
    },
)
