import rclpy
from pynput import keyboard
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import os

class SnapshotNode(Node):

    def __init__(self):
        super().__init__('dp_test')

        self.cam_sub = self.create_subscription(Image, 'image', self.cam_callback, 1)
        self.img_pub = self.create_publisher(Image, 'snapshot', 1)
        self.bridge = CvBridge()

        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        self.img = None
        self.i = 0
        self.folder = os.path.join('/home', 'ubuntu', 'image_collection')

    def cam_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg)

    def on_press(self, key):
        if cv2.imwrite(os.path.join(self.folder, 'image_' + str(self.i) + '.jpg'), self.img):
            self.i += 1
            print('image saved')
        else:
            print('image not saved')

        


def main(args=None):
    rclpy.init(args=args)
    data_collector = SnapshotNode()
    rclpy.spin(data_collector)
    data_collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()