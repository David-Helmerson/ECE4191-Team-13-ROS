import rclpy
import time
import math
from rclpy.node import Node
from project_interfaces.msg import RobotVelocity
from project_interfaces.srv import PoseRequest

class EncoderPredictorNode(Node):
    """
    Provides a service that predicts the robot position at a given time
    """
    def __init__(self):
        super().__init__('encoder_predictor')

        self.pose = (0.0, 0.0, 0.0)  # Pose is x, y, th
        self.v, self.w = 0.0, 0.0  # Last recorded velocity
        self.t = time.time()  # Time of last recorded velocity

        # Important objects
        self.sub = self.create_subscription(RobotVelocity, 'encoder_vel', self.vel_callback, 10)
        self.pose_srv = self.create_service(PoseRequest, 'get_pose', self.request_callback)

    def predict_pose(self, t):
        if self.w == 0:
            x = self.pose[0] + self.v*math.cos(self.pose[2])
            y = self.pose[1] + self.v*math.sin(self.pose[2])
            th = self.pose[2]
        else:
            phi = self.w*(t-self.t)/2
            a = 2*self.v/self.w*math.sin(phi)
            x = self.pose[0] + a*math.cos(self.pose[2]+phi)
            y = self.pose[1] + a*math.sin(self.pose[2]+phi)
            th = self.pose[2] + 2*phi
        return x, y, th

    def vel_callback(self, msg):
        # update pose based on velocity
        self.pose = self.predict_pose(time.time())
        self.v, self.w = msg.v, msg.w
        self.t = time.time()

    def request_callback(self, request, response):
        # Extrapolate pose from velocity and request time
        response.pose.x, response.pose.y, response.pose.th = self.predict_pose(request.time)
        return response

def main(args=None):
    rclpy.init(args=args)
    encoder_predictor = EncoderPredictorNode()
    rclpy.spin(encoder_predictor)
    encoder_predictor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()