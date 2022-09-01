import math
import rclpy
from rclpy.node import Node
from navigation.navigation.nav_helpers import *
from geometry_msgs.msg import Twist
from project_interfaces.msg import SerialCommand, WaypointPath
from nav_msgs.msg import OccupancyGrid


class PlannerNode(Node):
    """
    Ros integration of given code
    """
    def __init__(self):
        super().__init__('planner')
        
        # ROS2 parameters
        self.declare_parameter('waypoint_freq', 30.0)
        self.declare_parameter('lookahead_dist', 1.0)
        self.declare_parameter('stop_dist', 0.1)
        self.declare_parameter('max_lookahead', 0)

        # Important objects
        self.path_waypoints = None
        self.robot_pose = None
        self.map = None

        self.pose_sub = self.create_subscription(Twist, 'robot_pose', self.pose_callback)
        self.vel_pub = self.create_publisher(SerialCommand, 'command_send', 10)

        waypoint_freq = self.get_parameter('waypoint_freq').get_parameter_value().double_value
        self.vel_pub_timer = self.create_timer(1/waypoint_freq, self.tentacle_callback)

    # Update path upon any relevant information being updated
    def pose_callback(self, msg):
        self.robot_pose = msg
        self.update_path()

    def tentacle_callback(self):
        #TODO: Pub lin/ang velocity
        pass

    


        

def main(args=None):
    rclpy.init(args=args)
    planner = PlannerNode()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
