import rclpy
import torch
import torchvision.transforms as transforms
import os
import time

from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from project_interfaces.msg import MarbleArray, MarblePos
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import math

class YOLONode(Node):
    """
    Node to identify magnetic marble positions relative to robot, via use of custom YOLO model

    ...

    Parameters
    ----------
    marble_rad : float
        Radius of target marbles in meters

    box_buffer : int
        Assumed overestimation of bounding box width/height, in pixels
    
    model_name : string
        Name of YOLO model file stored in perception/models

    Topics
    ------
    image : L{sensor_msgs.Image}
        Subscribed camera image with resolution 640x480

    marbles : L{project_interfaces.MarbleArray}
        Output positions of marbles in robot frame (presumed axle center footprint)

    """
    def __init__(self):
        super().__init__('yolo_node')

        # ROS2 Parameters
        self.declare_parameter('marble_rad', 0.02)
        self.declare_parameter('box_buffer', 4)
        self.declare_parameter('model_name', '500kb_targets.pt')
        self.marble_rad = self.get_parameter('marble_rad').get_parameter_value().double_value
        self.box_buffer = self.get_parameter('box_buffer').get_parameter_value().double_value

        # Important ROS objects
        self.img_sub = self.create_subscription(Image, 'image', self.img_callback, 1)
        self.pub = self.create_publisher(MarbleArray, 'marbles', 10)
        self.bridge = CvBridge()

        # Load model
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        model_path = os.path.join(get_package_share_directory('perception'), 'models', model_name)
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=False)
        self.get_logger().info('Model loaded')

        # Camera vars
        self.focal_length = 0.25  # Focal length in meters
        self.principle_x = 320  # Principle points in pixels
        self.principle_y = 240
        self.camera_xyz = np.array([0, 0.2, 0.05])
        self.camera_th = -math.pi*3/180

        # Derived vars
        s, c = math.sin(self.camera_th), math.cos(self.camera_th)
        self.rot = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


    def img_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)

        # Detect marbles
        t0 = time.time()
        df = self.model(img).pandas().xyxy[0]
        self.get_logger().info("inference time: {time:.5f}s".format(time = time.time()-t0))
        print(df)

        # Process marbles
        out_msg = MarbleArray()
        box_arr = df[df['class'] == 0].to_numpy()[:, :-3]
        marble_arr = []
        for box in box_arr:
            # Find marble location in camera frame, where z is depth
            w, h = box[2] - box[0], box[3] - box[1] 
            px, py, pd = box[0] + w/2, box[1] + h/2, (w+h)/2 - self.box_buffer
            print('pixel xyd', px, py, pd)
            cxyz = np.array([(px - self.principle_x), (py - self.principle_y), self.focal_length])*self.marble_rad/pd
            print('cxyz', cxyz)
            # Transform into planar x, z coordinates
            xyz = self.rot.dot(cxyz) + self.camera_xyz
            print('xyz', xyz)
            marble = MarblePos()
            marble.x, marble.z = xyz[0], xyz[2]
            marble_arr.append(marble)
        
        out_msg.data = marble_arr
        self.pub.publish(out_msg)
                
                
def main(args=None):
    with torch.no_grad():
        rclpy.init(args=args)
        yolo = YOLONode()
        rclpy.spin(yolo)
        yolo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()