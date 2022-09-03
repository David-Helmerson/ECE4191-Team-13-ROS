import rclpy
import time
import math
from rclpy.node import Node
from project_interfaces.msg import RobotVelocity, RobotPose
from project_interfaces.srv import PoseRequest

class EncoderPredictorNode(Node):
    """
    Provides a service that predicts the robot position at a given time
    """
    def __init__(self):
        super().__init__('encoder_predictor')

        self.pose = [0, 0, 0]  # Pose is x, y, th
        self.v, self.w = 0, 0  # Last recorded velocity
        self.t = time.time()  # Time of last recorded velocity

        # Important objects
        self.sub = self.create_subscription(RobotVelocity, 'encoder_vel', self.vel_callback, 10)
        self.pose_srv = self.create_service(PoseRequest, 'get_pose', self.request_callback)

    def vel_callback(self, msg):
        # update pose based on velocity
        self.v, self.w = msg.v, msg.w
        self.t = time.time()

    def request_callback(self, request, response):
        # Extrapolate pose from velocity and request time
        phi = self.w*(request.time-self.t)/2
        a = 2*self.v/self.w*math.sin(phi)
        response.x = self.pose[0] + a*math.cos(self.pose[2]+phi)
        response.y = self.pose[1] + a*math.sin(self.pose[2]+phi)
        response.th = self.pose[2] + 2*phi
        return response

def main(args=None):
    rclpy.init(args=args)
    encoder_predictor = EncoderPredictorNode()
    rclpy.spin(encoder_predictor)
    encoder_predictor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()