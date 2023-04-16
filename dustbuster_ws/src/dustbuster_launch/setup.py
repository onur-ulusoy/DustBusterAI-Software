import os
from glob import glob
from setuptools import setup

package_name = 'dustbuster_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
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
