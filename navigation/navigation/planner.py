import math
import rclpy
from rclpy.node import Node
from navigation.navigation.nav_helpers import *
from geometry_msgs.msg import Twist
from project_interfaces.msg import SerialCommand, WaypointPath
from nav_msgs.msg import OccupancyGrid


class PlannerNode(Node):
    """
    Handles path waypoint generation within a given map, and provides pure pursuit following of path.
    ...

    Parameters
    ----------
    waypoint_freq : double
        frequency that instantaneous waypoint commands are published to serial topic
    lookahead_dist : double
        lookahead distance for the pure pursuit algorithm
    stop_dist : double
        distance within a final waypoint that the robot should stop at
    max_lookahead : int
        maximum number of waypoints that the pure pursuit algorithm will consider

    Topics
    ------
    robot_pose : L{geometry_msgs.Twist} message
        Subscribed updates for the robot's position
    occupancy_grid : L{nav_msgs.OccupancyGrid} message
        Subscribed updates for the global ocupancy grid
    path_override : L{project_interfaces.WaypointPath} message
        Manual setting of waypoint path
    command_send : L{project_interfaces.SerialCommand} message
        Published topic for instantaneous goal position commands

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
        self.map_sub = self.create_subscription(Twist, 'occupancy_grid', self.pose_callback)
        self.path_sub = self.create_subscription(WaypointPath, 'path_override', self.path_callback)
        self.wp_pub = self.create_publisher(SerialCommand, 'command_send', 10)

        waypoint_freq = self.get_parameter('waypoint_freq').get_parameter_value().double_value
        self.wp_pub_timer = self.create_timer(1/waypoint_freq, self.pp_callback)

    # Update path upon any relevant information being updated
    def pose_callback(self, msg):
        self.robot_pose = msg
        self.update_path()

    def map_callback(self, msg):
        self.map = msg
        self.update_path()

    def path_callback(self, msg):
        self.path_waypoints = msg

    def update_path(self):
        """
        Update path_waypoints based on current robot pose and map
        """
        if self.robot_pose is None or self.map is None:
            self.get_logger().warning('Insufficient information initialized for waypoint generation')

        pass

    def pp_callback(self):
        """
        Calculates instantaneous waypoint to move towards according to pure pursuit where applicable,
        otherwise moves to the last waypoint if within lookahead distance, or otherwise to the first waypoint
        TODO: Remove waypoints up to last_wp?
        """

        # Cannot perform pure pursuit if there is not path to follow
        if self.path_waypoints is None:
            self.get_logger().warning('Insufficient information initialized for pure pursuit planning')
            return None

        # Calculate waypoint in world frame
        bot_x, bot_y = self.robot.pose.linear.x, self.robot.pose.linear.y
        lookahead_dist = self.get_parameter('lookahead_dist').get_parameter_value().double_value
        ml, last_wp = self.get_parameter('max_lookahead').get_parameter_value().integer_value
        wp = pp_waypoint(self.path_waypoints, (bot_x, bot_y), lookahead_dist, ml if ml else math.inf)


        if wp is None:
            # Check if final waypoint is within pathing distance
            wp = self.path_waypoints.path[-1]
            final_dist2 = (wp[0] - bot_x)**2 + (wp[0] - bot_x)**2 
            if final_dist2 < lookahead_dist**2:
                stop_dist = self.get_parameter('stop_dist').get_parameter_value().double_value
                # If final waypoint is within stopping distance don't move
                if final_dist2 < stop_dist**2: 
                    self.get_logger().info('Final waypoint reached, stopping pathing')
                    return None
            # Otherwise path to first waypoint
            else: wp = self.path_waypoints.path[0]

        # Translate waypoint to robot frame and formulate command
        command = SerialCommand()
        command.id = 0   
        yaw = self.robot_pose.angular.z
        command.p1 = (wp[0] - bot_x)*math.cos(yaw) + (wp[1] - bot_y)*math.sin(yaw)
        command.p2 = (wp[1] - bot_y)*math.cos(yaw) + (wp[0] - bot_x)*math.sin(yaw)
        self.wp_pub.publish(command)

    


        

def main(args=None):
    rclpy.init(args=args)
    planner = PlannerNode()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
