from setuptools import setup

package_name = 'dustbuster_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ekf_fusion.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Onur Ulusoy',
    maintainer_email='onurulusoys4@gmail.com',
    description='ROS2 localization package for DustBusterAI',
    license='MIT',
    tests_require=['pytest']
)
