import time
import rclpy
from rclpy.node import Node
from navigation.nav_helpers import *
from geometry_msgs.msg import Twist
from project_interfaces.msg import RobotPose, SerialCommand, UltrasonicDistances, Waypoint
from project_interfaces.srv import PoseRequest

class TentacleNode(Node):
    """
    Ros integration of given code, I'd write a docstring if I knew exactly what this did and why.
    """
    def __init__(self):
        super().__init__('tentacle_planner')
        
        # ROS2 parameters
        self.declare_parameter('freq', 15.0)
        self.declare_parameter('obstacle_threshold', 0.05)
        self.freq = self.get_parameter('freq').get_parameter_value().double_value

        # Important objects
        self.pose = None
        self.goal_wp = None
        self.out_msg = SerialCommand()
        self.last_msg = SerialCommand()
        self.last_send_time = -math.inf
        self.tentacle_planner = TentaclePlanner(1/self.freq, 2, 1, 0.00)  # Plans head for twice the node's operating freq, just incase

        self.goal_sub = self.create_subscription(Waypoint, 'goal_waypoint', self.goal_callback, 10)
        self.pose_sub = self.create_subscription(RobotPose, 'pose_est', self.pose_callback, 10)
        self.us_sub = self.create_subscription(UltrasonicDistances, 'ultrasonic_distances', self.us_callback, 10)
        self.cmd_pub = self.create_publisher(SerialCommand, 'command_send', 10)
        self.tentacle_timer = self.create_timer(1/self.freq, self.tentacle_callback)

    def goal_callback(self, msg): self.goal_wp = msg
    def pose_callback(self, msg): self.pose = msg
    
    def us_callback(self, msg):
        thresh = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        self.tentacle_planner.left, self.tentacle_planner.right = msg.left < thresh, msg.right < thresh

    def tentacle_callback(self):
        #self.get_logger().info('tentacle callback ' + str(self.last_send_time - time.time()) + ' ' + )
        # and ((self.out_msg != self.last_msg) or (time.time() - self.last_send_time  > 1))
        if self.goal_wp is not None and self.pose is not None:
            self.get_logger().info('tentacle pub')
            v, w = self.tentacle_planner.plan(self.goal_wp.x, self.goal_wp.y, 0, self.pose.x, self.pose.y, self.pose.th)
            self.out_msg.id, self.out_msg.p1, self.out_msg.p2 = 8, v, w
            self.cmd_pub.publish(self.out_msg)
            self.last_msg, self.last_send_time = self.out_msg, time.time()

    
def main(args=None):
    rclpy.init(args=args)
    tentacle_planner = TentacleNode()
    rclpy.spin(tentacle_planner)
    tentacle_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
