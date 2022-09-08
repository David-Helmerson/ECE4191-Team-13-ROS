import time
import rclpy
from rclpy.node import Node
from navigation.nav_helpers import *
from geometry_msgs.msg import Twist
from project_interfaces.msg import RobotPose, SerialCommand, UltrasonicDistances, Waypoint
from project_interfaces.srv import PoseRequest

class StupidNode(Node):
    """
    Ros integration of given code, I'd write a docstring if I knew exactly what this did and why.
    """
    def __init__(self):
        super().__init__('stupid_planner')
        
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
        self.stupid_planner = StupidPlanner()
        self.state_2_time = 1.0

        self.goal_sub = self.create_subscription(Waypoint, 'goal_waypoint', self.goal_callback, 10)
        self.pose_sub = self.create_subscription(RobotPose, 'pose_est', self.pose_callback, 10)
        self.us_sub = self.create_subscription(UltrasonicDistances, 'ultrasonic_distances', self.us_callback, 10)
        self.cmd_pub = self.create_publisher(SerialCommand, 'command_send', 10)
        self.stupid_timer = self.create_timer(1/self.freq, self.stupid_callback)

    def goal_callback(self, msg): self.goal_wp = msg
    def pose_callback(self, msg): self.pose = msg
    
    def us_callback(self, msg):
        thresh = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        self.stupid_planner.left, self.stupid_planner.right = msg.left < thresh, msg.right < thresh

    def stupid_callback(self):
        self.get_logger().info('state ' + str(self.stupid_planner.state))
        # and ((self.out_msg != self.last_msg) or (time.time() - self.last_send_time  > 1))
        if self.goal_wp is not None and self.pose is not None and (self.stupid_planner.state != 2):
            #self.get_logger().info('stupid pub')
            v, w = self.stupid_planner.plan(self.pose.x, self.pose.y, self.pose.th, self.goal_wp.x, self.goal_wp.y)
            self.out_msg.id, self.out_msg.p1, self.out_msg.p2 = 8, float(v), float(w)
            self.cmd_pub.publish(self.out_msg)
            self.last_msg, self.last_send_time = self.out_msg, time.time()

        if self.stupid_planner.state == 2 and (time.time() - self.stupid_planner.state_2_time) > self.state_2_time:
            self.stupid_planner.state = 0


    
def main(args=None):
    rclpy.init(args=args)
    stupid_planner = StupidNode()
    rclpy.spin(stupid_planner)
    stupid_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
