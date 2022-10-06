from sys import displayhook
import os
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


class MarbleDataCollectorNode(Node):

    def __init__(self):
        super().__init__('dp_test')

        self.cam_sub = self.create_subscription(Image, 'snapshot', self.image_callback, 1)
        self.pix_buffer = 5
        self.out_folder = '~/sampling_out'

        self.bridge = CvBridge()

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        # Detect edges
        edges = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.GaussianBlur(edges, (5, 5), 0)
        edges = cv2.Canny(edges, 35, 125)
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 30, param1=50, param2=30, minRadius=3, maxRadius=50)

        img_saved = False
        img_i = 0
        label_file = None
        self.get_logger().info(str(circles))
        if circles:
            for c in circles[0]:
                self.get_logger().info(str(c))
                x0, y0 = (int(c[i] - c[2] - self.pix_buffer) for i in [0, 1])
                x1, y1 = (int(c[i] + c[2] + self.pix_buffer) for i in [0, 1])
                self.get_logger().info(' '.join([str(x) for x in [x0, x1, y0, y1]]))
                if x0 > 0 and x1 < 640 and y0 > 0 and y1 < 240:
                    cv2.imshow('image', img[x0:x1, y0:y1])
                    cv2.waitKey(0)
                    '''
                    inp = int(input('1: magnetic marble, 2: non-magnetic marble, 3: nothing: '))
                    if inp in [1, 2]:
                        if not img_saved:
                            img_name = 'image_' + str(img_i) + '.jpg'
                            cv2.imwrite(os.path.join(self.out_folder, 'images', img_name), img)
                            label_file = open('image_' + str(img_i) + '.txt', 'w')

                        label_file.write(' '.join([inp, c[0], c[1], c[2], c[2]]) + '\n')
                    '''


def main(args=None):
    rclpy.init(args=args)
    data_collector = MarbleDataCollectorNode()
    rclpy.spin(data_collector)
    data_collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()