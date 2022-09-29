from sys import displayhook
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
from project_interfaces.msg import RobotPose, RobotVelocity
import cv2
import numpy as np
import time
from perception.cloud_tools import create_cloud, create_cloud_from_list

# ROS2 camera reading node is done via: ros2 run image_tools cam2image
# Create an odom frame with: ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world odom


class DPTestNode(Node):

    def __init__(self):
        super().__init__('dp_test')

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.cam_sub = self.create_subscription(Image, 'image', self.image_callback, 10)
        self.cam_pub = self.create_publisher(Image, 'test_image', 10)
        self.pose_pub = self.create_publisher(RobotPose, 'test_pose', 10)
        self.vel_pub = self.create_publisher(RobotVelocity, 'test_vel', 10)
        self.bridge = CvBridge()

        self.last_image = None
        self.image1 = None
        self.pos1 = None
        self.image2 = None
        self.pos2 = None

    def image_callback(self, msg):
        self.last_image = msg
    
    def timer_callback(self):
        self.vel_pub.publish(RobotVelocity())
        if self.last_image is not None:
            if self.image1 is None:
                self.image1 = self.last_image
                cv2.imshow('img', self.bridge.imgmsg_to_cv2(self.image1, 'rgb8'))
                self.pos1 = RobotPose()
                self.pos1.x, self.pos1.y, self.pos1.th = input("Enter position of photo 1 (x,y,th): ").split(',')
                self.pose_pub.publish(self.pos1)
                self.cam_pub.publish(self.image1)

            elif self.image2 is None:
                self.image2 = self.last_image
                cv2.imshow('img', self.bridge.imgmsg_to_cv2(self.image2, 'rgb8'))
                self.pos2 = RobotPose()
                self.pos2.x, self.pos2.y, self.pos2.th = input("Enter position of photo 2 (x,y,th): ").split(',')
                self.pose_pub.publish(self.pos1)
                self.cam_pub.publish(self.image2)
                self.image1 = None


def main(args=None):
    rclpy.init(args=args)
    dp_test = DPTestNode()
    rclpy.spin(dp_test)
    dp_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()