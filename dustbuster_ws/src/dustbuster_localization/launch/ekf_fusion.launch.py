from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_fusion',
            parameters=[
                {
                    'imu0': '/imu',
                    'odom0': '/odom',
                    'odom_frame': 'odom',
                    'base_link_frame': 'base_link',
                    'world_frame': 'odom',
                    'two_d_mode': True,
                    'map_frame': '',
                    'transform_time_offset': 0.0,
                    'odom0_config': [False, False, False,
                                     False, False, False,
                                     True, True, True,
                                     False, False, True,
                                     False, False, False],
                    'imu0_config': [False, False, False,
                                    True, True, True,
                                    False, False, False,
                                    False, False, False,
                                    True, True, True],
                    'odom0_differential': False,
                    'imu0_differential': False,
                    'odom0_relative': False,
                    'imu0_relative': False,
                    'imu0_remove_gravitational_acceleration': True,
                    'publish_tf': True,
                    'publish_acceleration': False,
                    'odom0_process_noise_covariance': [1e-5, 0, 0, 0, 0, 0,
                                                        0, 1e-5, 0, 0, 0, 0,
                                                        0, 0, 1e-5, 0, 0, 0,
                                                        0, 0, 0, 1e-5, 0, 0,
                                                        0, 0, 0, 0, 1e-5, 0,
                                                        0, 0, 0, 0, 0, 1e-5],
                    'imu0_process_noise_covariance': [1e-5, 0, 0, 0, 0, 0,
                                                      0, 1e-5, 0, 0, 0, 0,
                                                      0, 0, 1e-5, 0, 0, 0,
                                                      0, 0, 0, 1e-5, 0, 0,
                                                      0, 0, 0, 0, 1e-5, 0,
                                                      0, 0, 0, 0, 0, 1e-5]
                }
            ],
            output='screen'
        )
    ])
