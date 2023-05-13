import os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import yaml
import argparse

class MapSaver(Node):

    def __init__(self, map_name, map_path):
        super().__init__('map_saver')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10
        )
        self.map_name = map_name
        self.map_path = map_path
        self.map_saved = False

    def listener_callback(self, msg):
        # Extract data from message
        data = np.array(msg.data, dtype=np.int8)
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # Reshape data and flip y-axis to match OpenCV image format
        data = data.reshape((height, width))[::-1, :]

        # Convert data to image
        image = np.zeros_like(data, dtype=np.uint8)
        image[data==-1] = 127  # Unknown areas
        image[data==0] = 255   # Free areas
        image[data==100] = 0   # Occupied areas

        # Save image as .pgm file
        cv2.imwrite(os.path.join(self.map_path, f'{self.map_name}.pgm'), image)

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
        with open(os.path.join(self.map_path, f'{self.map_name}.yaml'), 'w') as f:
            yaml.dump(metadata, f)

        self.get_logger().info(f'Map data saved to {self.map_name}.pgm and {self.map_name}.yaml')
        self.map_saved = True

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Save map to file')
    parser.add_argument('map_name', metavar='map_name', type=str, help='name of the map to save')
    parser.add_argument('--map_path', default='.', help='directory to save the map files')

    # If args is not None, it means we're calling this function programmatically, so we parse these arguments
    # If args is None, it means we're calling this function from the command line, so we parse sys.argv
    parsed_args = parser.parse_args() if args is None else parser.parse_args(args)

    map_saver = MapSaver(parsed_args.map_name, parsed_args.map_path)

    executor = SingleThreadedExecutor()
    executor.add_node(map_saver)

    while rclpy.ok():
        rclpy.spin_once(map_saver, executor=executor)
        if map_saver.map_saved:
            map_saver.get_logger().info("Map saved, map saver shutting down...")
            map_saver.destroy_node()
            rclpy.shutdown()
            break

if __name__ == '__main__':
    main()
