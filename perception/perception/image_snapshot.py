import rclpy
from pynput import keyboard
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class SnapshotNode(Node):

    def __init__(self):
        super().__init__('dp_test')

        #self.cam_sub = self.create_subscription(Image, 'image', self.cam_callback, 1)
        self.img_pub = self.create_publisher(Image, 'snapshot', 1)
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture(0)

        self.listener = keyboard.Listener(on_press=self.on_press)

    def on_press(self):
        _, frame = self.capture.read()
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(frame))


def main(args=None):
    rclpy.init(args=args)
    data_collector = SnapshotNode()
    rclpy.spin(data_collector)
    data_collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()