import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
import math
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose


class WaypointsNavigation(Node):

    def __init__(self):
        super().__init__('waypoints_navigation_node')
        self.subscription = self.create_subscription(
            String,
            'tsp_optimal_tour',
            self.waypoints_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = []
        self.marker_array_pub = self.create_publisher(MarkerArray, 'waypoints_markers', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoints_labels', 10)
        self.speed_threshold = 0.6  # Threshold for robot speed (m/s) to detect deacceleration
        self.last_speed = 0.0  # Initialize robot speed
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10 )
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_pose = None
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.current_goal_index = 0
        self.robot_started_moving = False

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_current_pose(self):
        return self.current_pose    
    
    def get_transformed_pose(self, pose):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            transformed_pose = do_transform_pose(pose, transform)
            return transformed_pose
        except (LookupException, ExtrapolationException):
            self.get_logger().warning('Unable to get the map to odom transform. Using current pose instead.')
            return self.current_pose
    
    def waypoints_callback(self, msg):
        data = msg.data.strip().split(';')
        for point in data:
            coords = point.split(',')
            x, y = float(coords[0].strip()), float(coords[1].strip())
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.orientation.w = 1.0
            self.waypoints.append(waypoint)
            
            # Debug prints
            self.get_logger().info(f"Received waypoint: x={x}, y={y}")
            
        self.get_logger().info(f"Number of waypoints: {len(self.waypoints)}")
        self.publish_markers()
        self.publish_labels()
        self.send_navigation_goal()

    def distance_to_goal(self, current_pose, goal_pose):
        dx = goal_pose.pose.position.x - current_pose.position.x
        dy = goal_pose.pose.position.y - current_pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def cmd_vel_callback(self, msg):
        if self.current_goal_index < len(self.waypoints) - 1:
            current_pose = self.get_current_pose()
            transformed_pose = self.get_transformed_pose(current_pose)
            distance = self.distance_to_goal(transformed_pose, self.waypoints[self.current_goal_index])
            distance_threshold = 1.5
            print(distance)
            print(current_pose.position.x)
            print(self.waypoints[self.current_goal_index])
            if distance < distance_threshold:
                self.current_goal_index += 1
                self.send_navigation_goal()
                self.robot_started_moving = False

                
    def publish_twist(self):
        twist = Twist()
        twist.linear.x = self.speed_threshold
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Published cmd_vel message')

    def send_navigation_goal(self):
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.waypoints[self.current_goal_index]  # Use only the current goal waypoint
        self.action_client.send_goal_async(goal_msg)

        self.get_logger().info(f'Sent navigation goal to waypoint {self.current_goal_index + 1}.')
    
    def publish_markers(self):
        marker_array = MarkerArray()
        for idx, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.pose.position.x
            marker.pose.position.y = waypoint.pose.position.y
            marker.pose.orientation.w = 1.0
            marker.id = idx
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.marker_array_pub.publish(marker_array)
    
    def publish_labels(self):
        marker_array = MarkerArray()

        for i, waypoint in enumerate(self.waypoints[:-1]):  # Exclude the last waypoint
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.ns = 'waypoint_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = waypoint.pose.position
            text_marker.pose.position.z += 0.5  # Adjust the height of the text label
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.3  # Adjust the size of the text
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = str(i + 1)  # Set the text to the waypoint index, add 1
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    waypoints_navigation_node = WaypointsNavigation()
    rclpy.spin(waypoints_navigation_node)

    waypoints_navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
