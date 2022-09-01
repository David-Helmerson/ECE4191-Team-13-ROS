import math
import rclpy
from rclpy.node import Node
from navigation.navigation.nav_helpers import *
from geometry_msgs.msg import Twist
from project_interfaces.msg import SerialCommand, UltrasonicDistances, Waypoint


class PlannerNode(Node):
    """
    Ros integration of given code
    """
    def __init__(self):
        super().__init__('planner')
        
        # ROS2 parameters
        self.declare_parameter('freq', 30.0)
        self.declare_parameter('obstacle_threshold', 0.05)
        self.freq = self.get_parameter('freq').get_parameter_value().double_value

        # Important objects
        self.robot_pose = None
        self.goal_wp = None
        self.planner = TentaclePlanner(1/self.freq, 1, 1, 0)

        self.pose_sub = self.create_subscription(Twist, 'robot_pose', self.pose_callback)
        self.goal_sub = self.create_subscription(Waypoint, 'goal_waypoint', self.goal_callback)
        self.us_sub = self.create_subscription(UltrasonicDistances, 'us_dists', self.us_callback)
        self.vel_pub = self.create_publisher(SerialCommand, 'command_send', 10)
        self.vel_pub_timer = self.create_timer(1/self.freq, self.tentacle_callback)

    # Update path upon any relevant information being updated
    def pose_callback(self, msg):
        self.robot_pose = [msg.linear.x, msg.linear.y, msg.angular.z]

    def goal_callback(self, msg):
        self.goal_wp = msg

    def us_callback(self, msg):
        thresh = self.get_parameter('object_threshold').get_parameter_value().double_value
        self.planner.left, self.planner.right = msg.left < thresh, msg.right < thresh

    def tentacle_callback(self):
        if self.robot_pose is not None and self.goal_wp is not None:
            v, w = TentaclePlanner.plan(self.goal_wp.x, self.goal_wp.y, 
                                        self.robot_pose[0], self.robot_pose[1], self.robot_pose[2])
            out_msg = SerialCommand()
            out_msg.id, out_msg.p1, out_msg.p2 = 101, v, w
            self.vel_pub.publish(out_msg)

    


        

def main(args=None):
    rclpy.init(args=args)
    planner = PlannerNode()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
