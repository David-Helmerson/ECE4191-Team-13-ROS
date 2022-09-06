import time
import rclpy
from rclpy.node import Node
from project_interfaces.msg import Waypoint
from project_interfaces.srv import PoseRequest

class WaypointManagerNode(Node):

    def __init__(self):
        super().__init__('waypoint_manager')

        # ROS2 Parameters
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('distance_thresh', 0.05)
        self.freq = self.get_parameter('frequency').get_parameter_value().double_value

        # Important objects
        self.wp_list = [(1.0, 1.0), (2.0, 2.0)]

        self.pose_client = self.create_client(PoseRequest, 'get_pose')
        while not self.pose_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('pose server not available, waiting..')
        self.pose_req = PoseRequest.Request()

        self.pub = self.create_publisher(Waypoint, 'goal_waypoint', 10)
        self.timer = self.create_timer(1/self.freq, self.timer_callback)
        self.msg = Waypoint()
        self.pub_wp()

    def pub_wp(self):
        self.msg.x, self.msg.y = self.wp_list[0][0], self.wp_list[0][1]
        self.pub.publish(self.msg)

    def get_pose(self):
        self.pose_req.time = time.time()
        self.future = self.pose_client.call_async(self.pose_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().pose

    def timer_callback(self):
        thresh = self.get_parameter('distance_thresh').get_parameter_value().double_value
        pose = self.get_pose()
        if (pose.x-self.wp_list[0][0])**2 + (pose.y-self.wp_list[0][1])**2 < thresh**2:
            del self.wp_list[0]
            self.pub_wp()



def main(args=None):
    rclpy.init(args=args)
    waypoint_manager = WaypointManagerNode()
    rclpy.spin(waypoint_manager)
    waypoint_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()