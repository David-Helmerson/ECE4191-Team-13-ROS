from sys import displayhook
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
        self.declare_parameter('display_matches', True)

        # Important objects
        self.cam_sub = self.create_subscription(Image, 'image', self.image_callback, 10)
        self.pose_sub = self.create_subscription(Twist, 'pose', self.image_callback, 10)
        self.match_pub = self.create_publisher(Image, 'matched_points', 10)
        self.bridge = CvBridge()
        self.orb = cv2.ORB_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.last_image = None
        self.last_pose = None
        self.camera_matrix = None
        self.base_camera_transform = np.matrix([[1, 0, 0, 0],
                                                [0, 0, 1, 0],
                                                [0, 1, 0, 0],
                                                [0, 0, 0, 1]])

    def feature_matcher(self, img1, img2):
        kp1, des1 = self.orb.detectAndCompute(img1, None)
        kp2, des2 = self.orb.detectAndCompute(img2, None)
        matches = None

        # Handle no detected features case
        if des1 is not None and des2 is not None:
            matches = self.matcher.match(des1, des2)
            matches = sorted(matches, key=lambda x: x.distance)

        # Publish matches image if specified
        if self.get_parameter('display_matches').get_parameter_value() and matches:
            match_img = cv2.drawMatches(img1,kp1,img2,kp2,matches,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            msg = self.bridge.cv2_to_imgmsg(match_img, 'rgb8')
            self.match_pub.publish(msg)

        return matches


    def triangulate(self, kp1, kp2, pose1, pose2):
        l = ((x, y, np.cos(r), np.sin(r)) for x, y, r in (pose1, pose2))
        T_list = (np.matrix([[c, s, 0, -x*c - y*s],
                            [-s, c, 0, x*s - y*c],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]]) for x, y, c, s in l)
        T1, T2 = [self.camera_matrix @ self.base_camera_transform @ T for T in T_list]

        kp1_3D = np.ones((3, kp1.shape[0]))
        kp2_3D = np.ones((3, kp2.shape[0]))
        kp1_3D[0], kp1_3D[1] = kp1[:, 0].copy(), kp1[:, 1].copy()
        kp2_3D[0], kp2_3D[1] = kp2[:, 0].copy(), kp2[:, 1].copy()
        X = cv2.triangulatePoints(T1[:3], T2[:3], kp1_3D[:2], kp2_3D[:2])
        X /= X[3]
        return X[:3], T1[:3] @ X, T2[:3] @ X


    def pose_callback(self, msg):
        self.last_pose = [msg.linear.x, msg.linear.y, msg.angular.z]

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        if self.last_image is not None:
            features = self.feature_matcher(img, self.last_image)
            #cv2.drawMatches(img, kp1,self.last_image,kp2, features, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    


        self.last_image = img
        



def main(args=None):
    rclpy.init(args=args)
    depth_perception = DepthPerceptionNode()
    rclpy.spin(depth_perception)
    depth_perception.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()