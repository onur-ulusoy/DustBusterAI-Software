#!/usr/bin/env python3

"""
driver.py

Driver script to publish joint velocities of the robot to necessary topics.

This script creates a ROS 2 node and publishes joint velocities of a robot in rad/s
to the topics `/left_joint_velocity` and `/right_joint_velocity`.

Usage: ros2 run dustbuster_nav driver --leftwh <LEFT_WHEEL_VELOCITY> --rightwh <RIGHT_WHEEL_VELOCITY>

Author: Onur Ulusoy
Date: 19.03.2023

This code is licensed under the MIT license.

"""

import argparse
import rclpy
from std_msgs.msg import Float64

def main(args=None):
    # Initialize the ROS 2 node
    rclpy.init(args=args)

    # Create a ROS 2 node
    node = rclpy.create_node('drive_node')

    # Parse the command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--leftwh', type=float, help='Angular velocity of the left wheel in rad/s')
    parser.add_argument('--rightwh', type=float, help='Angular velocity of the right wheel in rad/s')
    args = parser.parse_args()

    # Create publishers for the left and right wheel joint velocities
    publisher_leftwh = node.create_publisher(Float64, '/left_joint_velocity', 10)
    publisher_rightwh = node.create_publisher(Float64, '/right_joint_velocity', 10)

    # Set the initial joint velocities for the left and right wheels
    leftJointVelocity = Float64()
    leftJointVelocity.data = args.leftwh or 350.0  # Set the value of the angular velocity of the left wheel in rad/s

    rightJointVelocity = Float64()
    rightJointVelocity.data = args.rightwh or 0.0  # Set the value of the angular velocity of the right wheel in rad/s

    # Publish joint velocities in a loop until the node is shutdown
    while rclpy.ok():
        publisher_leftwh.publish(leftJointVelocity)
        node.get_logger().info('L Joint Publishing: %f' % leftJointVelocity.data)

        publisher_rightwh.publish(rightJointVelocity)
        node.get_logger().info('R Joint Publishing: %f' % rightJointVelocity.data)

    # Destroy the node explicitly
    node.destroy_node()

    # Shutdown the ROS 2 client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
