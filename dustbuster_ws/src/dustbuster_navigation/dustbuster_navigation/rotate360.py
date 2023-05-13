import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class Rotate360Node(Node):
    def __init__(self):
        super().__init__('rotate_360_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def rotate(self):
        twist = Twist()
        # Set the angular velocity in the z axis to 1.0
        twist.angular.z = 0.5
        # Calculate the sleep time needed for a full 360 degrees rotation
        sleep_time = 2*math.pi / twist.angular.z
        self.publisher_.publish(twist)
        time.sleep(sleep_time)  # sleep for the calculated time
        # Stop the robot after the rotation
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    rotate_360_node = Rotate360Node()

    # Call the rotate method directly after creating the node instance
    rotate_360_node.rotate()

    # No need to spin the node since we only want to publish once
    rotate_360_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
