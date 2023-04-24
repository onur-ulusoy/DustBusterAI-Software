import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import yaml

class CostmapSaver(Node):

    def __init__(self):
        super().__init__('costmap_saver')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.listener_callback,
            10
        )

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
        cv2.imwrite('map.pgm', image)

        # Save metadata as .yaml file
        metadata = {
            'image': 'map.pgm',
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
        with open('map.yaml', 'w') as f:
            yaml.dump(metadata, f)

        self.get_logger().info('Costmap data saved to map.pgm and map.yaml')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    costmap_saver = CostmapSaver()

    rclpy.spin(costmap_saver)

    costmap_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
