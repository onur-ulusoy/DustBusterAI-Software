import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped


class WaypointsNavigation(Node):

    def __init__(self):
        super().__init__('waypoints_navigation_node')
        self.subscription = self.create_subscription(
            String,
            'tsp_optimal_tour',
            self.waypoints_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.waypoints = []

    def waypoints_callback(self, msg):
        data = msg.data.replace(",", ";").split(";")
        x, y = float(data[0].strip()), float(data[1].strip())
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.orientation.w = 1.0
        self.waypoints.append(waypoint)

        # Send navigation goal after every new waypoint received
        self.send_navigation_goal()

    def send_navigation_goal(self):
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available.')
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints
        self.action_client.send_goal_async(goal_msg)

        self.get_logger().info('Sent navigation goal with waypoints.')



def main(args=None):
    rclpy.init(args=args)
    waypoints_navigation_node = WaypointsNavigation()
    rclpy.spin(waypoints_navigation_node)

    waypoints_navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
