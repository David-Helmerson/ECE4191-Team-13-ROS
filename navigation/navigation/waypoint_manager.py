import time
import rclpy
from rclpy.node import Node
from project_interfaces.msg import Waypoint, RobotPose
from project_interfaces.srv import PoseRequest

class WaypointManagerNode(Node):
    """
    Node that determines next waypoint to be pathed to from a fixed list.
    Waypoints changes are determined when robot reaches proximity of current goal

    Parameters
    ----------
    frequency: double
        Frequency of goal waypoint publishing
    distance_thresh: double
        Distance withing current goal waypoint that will consider it met

    Topics
    ------
    goal_waypoint: L{project_interfaces.Waypoint} message
        Current waypoint for the robot to path to

    Services
    --------
    get_pose: L{project_interfaces.PoseRequest} service
        Queries current time and recieves robot's predicted position based on latest velocity estimate

    """
    def __init__(self):
        super().__init__('waypoint_manager')

        # ROS2 Parameters
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('distance_thresh', 0.05)
        self.freq = self.get_parameter('frequency').get_parameter_value().double_value

        # Important objects
        self.wp_list = [(1.0, 1.0), (2.0, 2.0)]
        self.pose = None

        self.pose_sub = self.create_subscription(RobotPose, 'pose_est', self.pose_callback, 10)
        self.pub = self.create_publisher(Waypoint, 'goal_waypoint', 10)
        self.timer = self.create_timer(1/self.freq, self.timer_callback)
        self.msg = Waypoint()

        self.pub_wp()

    def pose_callback(self, msg): 
        self.pose = msg

    def pub_wp(self):
        self.msg.x, self.msg.y = self.wp_list[0][0], self.wp_list[0][1]
        self.pub.publish(self.msg)

    def timer_callback(self):
        if self.pose is not None:
            thresh = self.get_parameter('distance_thresh').get_parameter_value().double_value
            if (self.pose.x-self.wp_list[0][0])**2 + (self.pose.y-self.wp_list[0][1])**2 < thresh**2:
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