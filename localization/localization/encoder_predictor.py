import rclpy
import time
import math
from rclpy.node import Node
from project_interfaces.msg import RobotVelocity, RobotPose

class EncoderPredictorNode(Node):
    """
    Provides a service that predicts the robot position at a given time, based on rotary encoder velocity readings
    
    ...

    Topics
    ------
    encoder_vel: L{project_interfaces.RobotVelocity} message
        Linear and angular velocities of the robot as recieved from encoder data

    """
    def __init__(self):
        super().__init__('encoder_predictor')

        # Initial state of robot
        self.pose = (0.3, 0.2, math.pi/2)  # Pose is x, y, th
        self.v, self.w = 0.0, 0.0  # Last recorded velocity
        self.t = time.time()  # Time of last recorded velocity

        # ROS2 Parameters
        self.declare_parameter('frequency', 30.0)
        self.freq = self.get_parameter('frequency').get_parameter_value().double_value

        # Important objects
        self.sub = self.create_subscription(RobotVelocity, 'encoder_vel', self.vel_callback, 10)
        self.pose_pub = self.create_publisher(RobotPose, 'pose_est', 10)
        self.timer = self.create_timer(1/self.freq, self.timer_callback)
        self.out_msg = RobotPose()

    def predict_pose(self, t):
        """
        Predicts the pose of the robot at a given time

        ...

        Parameters
        ----------
        t: double
            requested time of robot pose prediction

        Returns
        -------
        x: double
            predicted x position of the robot in world frame
        y: double
            predicted y position of the robot in world frame
        th: double
            predicted yaw of the robot in world frame

        """
        # Predict linear motion
        if self.w == 0:
            x = self.pose[0] + self.v*(t-self.t)*math.cos(self.pose[2])
            y = self.pose[1] + self.v*(t-self.t)*math.sin(self.pose[2])
            th = self.pose[2]
        # Predict angular motion
        else:
            phi = self.w*(t-self.t)
            x = self.pose[0] + self.v*(t-self.t)*math.cos(self.pose[2])
            y = self.pose[1] + self.v*(t-self.t)*math.sin(self.pose[2])
            th = (self.pose[2] + phi) % (2*math.pi)
        return x, y, th

    def vel_callback(self, msg):
        # update pose based on velocity
        self.pose = self.predict_pose(time.time())
        self.v, self.w = msg.v, msg.w
        self.t = time.time()

    def timer_callback(self):
        self.out_msg.x, self.out_msg.y, self.out_msg.th = self.predict_pose(time.time())
        self.pose_pub.publish(self.out_msg)


def main(args=None):
    rclpy.init(args=args)
    encoder_predictor = EncoderPredictorNode()
    rclpy.spin(encoder_predictor)
    encoder_predictor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()