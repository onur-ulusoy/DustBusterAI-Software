import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import yaml
import argparse

class MapSaver(Node):

    def __init__(self, map_name):
        super().__init__('map_saver')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10
        )
        self.map_name = map_name

    def listener_callback(self, msg):
        # Extract data from message
        data = np.array(msg.data, dtype=np.int8)
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # Reshape data and flip y-axis to match OpenCV image format
        data = data.reshape((height, width))[::-1, :]

        # Convert data to image and save as .pgm file
        image = cv2.convertScaleAbs(data, alpha=(255.0/100.0))
        cv2.imwrite(f'{self.map_name}.pgm', image)

        # Save metadata as .yaml file
        metadata = {
            'image': f'{self.map_name}.pgm',
            'resolution': resolution,
            'origin': {
                'position': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'z': msg.info.origin.position.z,
                },
                'orientation': {
                    'x': msg.info.origin.orientation.x,
                    'y': msg.info.origin.orientation.y,
                    'z': msg.info.origin.orientation.z,
                    'w': msg.info.origin.orientation.w,
                }
            }
        }
        with open(f'{self.map_name}.yaml', 'w') as f:
            yaml.dump(metadata, f)

        self.get_logger().info(f'Map data saved to {self.map_name}.pgm and {self.map_name}.yaml')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Save map to file')
    parser.add_argument('map_name', metavar='map_name', type=str, help='name of the map to save')
    args = parser.parse_args()

    map_saver = MapSaver(args.map_name)

    rclpy.spin(map_saver)

if __name__ == '__main__':
    main()
