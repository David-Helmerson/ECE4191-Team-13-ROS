import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np

# ROS2 camera reading node is done via: ros2 run image_tools cam2image

class DepthPerceptionNode(Node):

    def __init__(self):
        super().__init__('depth_perception')

        # ROS2 parameters

        # Important objects
        self.cam_sub = self.create_subscription(Image, 'image', self.image_callback, 10)
        self.pose_sub = self.create_subscription(Twist, 'pose', self.image_callback, 10)
        self.bridge = CvBridge()
        self.last_image = None
        self.last_pose = None

    def pose_callback(self, msg):
        self.last_pose = [msg.linear.x, msg.linear.y, msg.angular.z]

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        if self.last_image is not None:
            pass

        self.last_image = img
        



def main(args=None):
    rclpy.init(args=args)
    depth_perception = DepthPerceptionNode()
    rclpy.spin(depth_perception)
    depth_perception.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()