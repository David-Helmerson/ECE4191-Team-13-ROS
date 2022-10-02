import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class MarbleDetectionNode(Node):
    """
    Description
    ...
    Parameters
    ----------
    None

    Topics
    ------
    """
    def __init__(self):
        super().__init__('minimal_publisher')

        # ROS Parameters
        self.declare_parameter('marble_radius', 0.005)  # Marble radius in meters
        self.declare_parameter('focal_length', 250)  # Focal length is some units???
        
        # Important ROS objects
        self.img_sub = self.create_subscription(Image, 'snap_image', self.image_callback, 10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        # Detect edges
        edges = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.GaussianBlur(edges, (5, 5), 0)
        edges = cv2.Canny(edges, 35, 125)
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 30, param1=50, param2=30, minRadius=3, maxRadius=50)
        
        if len(circles) > 0:
            x, y, r = self.circle[0, :][0], self.circle[0, :][1], self.circle[0, :][2]
            for i in range(len(circles)):
                circle_img = img[x[i]-r[i]:x[i]+r[i], y[i]-r[i]:y[i]+r[i]]
                # Detect if each circle is a magnetic marble
                if 4:
                    marble_r = 1
                    # Detect marble position
                    focal_length = self.get_parameter('focal_length').double_value()
                    real_radius = self.get_parameter('marble_radius').double_value()
                    dist = marble_r * focal_length / real_radius
            


def main(args=None):
    rclpy.init(args=args)
    marble_detection = MarbleDetectionNode()
    rclpy.spin(marble_detection)
    marble_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()