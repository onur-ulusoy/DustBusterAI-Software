from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the config directory
    config_dir = os.getcwd() + '/config'

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
