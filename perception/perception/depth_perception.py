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

class DepthPerceptionNode(Node):

    def __init__(self):
        super().__init__('depth_perception')

        # ROS2 parameters
        self.declare_parameter('display_matches', False)

        # Important objects
        self.cam_sub = self.create_subscription(Image, 'image', self.image_callback, 10)
        self.pose_sub = self.create_subscription(RobotPose, 'pose_est', self.image_callback, 10)
        self.vel_sub = self.create_subscription(RobotVelocity, 'encoder_vel', self.vel_callback, 10)
        self.match_pub = self.create_publisher(Image, 'matched_points', 10)
        self.points_pub = self.create_publisher(PointCloud2, 'depth_points', 10)
        self.bridge = CvBridge()
        self.orb = cv2.ORB_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.last_image = None
        self.image_pose = None
        self.image_time = None
        self.pose_time = None
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
            match_img = cv2.drawMatches(img1, kp1, img2, kp2, matches, None, 
                                        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            msg = self.bridge.cv2_to_imgmsg(match_img, 'rgb8')
            self.match_pub.publish(msg)

        # Arrange matches into arrays
        pts1 = np.array(kp1[m].pt for m in matches)
        pts2 = np.array(kp1[m].pt for m in matches)
        return pts1, pts2


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
        homog_pts = cv2.triangulatePoints(T1[:3], T2[:3], kp1_3D[:2], kp2_3D[:2])
        return homog_pts[:3]/homog_pts[3]


    def predict_pose(self, t):
        x = self.last_pose[0] + self.v*(t-self.pose_time)*np.cos(self.last_pose[2])
        y = self.last_pose[1] + self.v*(t-self.pose_time)*np.sin(self.last_pose[2])
        th = (self.last_pose[2] + self.w*(t-self.pose_time)) % (2*np.pi)
        return x, y, th


    def pose_callback(self, msg): self.last_pose = [msg.x, msg.y, msg.th]
    def vel_callback(self, msg): self.v, self.w = msg.v, msg.w

    def image_callback(self, msg):
        t = time.time()
        pose, img = self.predict_pose(t), self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        if self.last_image is not None:
            kp1, kp2 = self.feature_matcher(img, self.last_image)
            pts = self.triangulate(kp1, kp2, pose, self.image_pose)
            cloud_header = Header()
            cloud_header.stamp = self.get_clock().now().to_msg()
            cloud_header.frame_id = 'odom'
            cloud_out = create_cloud_from_list(cloud_header, pts)
            self.points_pub.publish(cloud_out)

        self.image_time, self.image_pose, self.last_image = t, pose, img


def main(args=None):
    rclpy.init(args=args)
    depth_perception = DepthPerceptionNode()
    rclpy.spin(depth_perception)
    depth_perception.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()