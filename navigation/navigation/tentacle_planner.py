import time
import rclpy
from rclpy.node import Node
from navigation.navigation.nav_helpers import *
from geometry_msgs.msg import Twist
from project_interfaces.msg import RobotPose, SerialCommand, UltrasonicDistances, Waypoint
from project_interfaces.srv import PoseRequest

class PlannerNode(Node):
    """
    Ros integration of given code
    """
    def __init__(self):
        super().__init__('planner')
        
        # ROS2 parameters
        self.declare_parameter('freq', 15.0)
        self.declare_parameter('obstacle_threshold', 0.05)
        self.freq = self.get_parameter('freq').get_parameter_value().double_value

        # Important objects
        self.goal_wp = None
        self.planner = TentaclePlanner(2/self.freq, 1, 1, 0)  # Plans head for twice the node's operating freq, just incase

        self.pose_client = self.create_client(PoseRequest, 'get_pose')
        while not self.pose_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('pose server not available, waiting..')
        self.pose_req = PoseRequest()

        self.goal_sub = self.create_subscription(Waypoint, 'goal_waypoint', self.goal_callback)
        self.us_sub = self.create_subscription(UltrasonicDistances, 'us_dists', self.us_callback)
        self.cmd_pub = self.create_publisher(SerialCommand, 'command_send', 10)
        self.tentacle_timer = self.create_timer(1/self.freq, self.tentacle_callback)

    def get_pose(self):
        self.pose_req.time = time.time()
        self.future = self.pose_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().pose

    def goal_callback(self, msg):
        self.goal_wp = msg

    def us_callback(self, msg):
        thresh = self.get_parameter('object_threshold').get_parameter_value().double_value
        self.planner.left, self.planner.right = msg.left < thresh, msg.right < thresh

    def tentacle_callback(self):
        if self.goal_wp is not None:
            pose = self.get_pose()
            v, w = self.planner.plan(self.goal_wp.x, self.goal_wp.y, 0, pose.x, pose.y, pose.th)
            out_msg = SerialCommand()
            out_msg.id, out_msg.p1, out_msg.p2 = 101, v, w
            self.cmd_pub.publish(out_msg)

    


        

def main(args=None):
    rclpy.init(args=args)
    planner = PlannerNode()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
