import math
import time
import rclpy
from rclpy.node import Node
from project_interfaces.msg import SerialCommand, UltrasonicDistances, RobotPose, MarbleArray

class SImplestStateMachineNode(Node):
    """
    Simplest central decision making node for the robot, based on a finite state machine.

    States
    ------

    ...

    Parameters
    ----------
    A lot

    Topics
    ------
    pose_est: L{project_interfaces.RobotPose}
        Publishes serial commands to be sent to PSoC

    marbles: L{project_interfaces.MarbleArray}
        Target marble x/z positions relative to robot

    ultrasonic_distances: L{project_interfaces.UltrasonicDistances}
        Recieves distances read by left/right ultransonic distance meters

    command_send: L{project_interfaces.SerialCommand}
        Publishes serial commands to be sent to PSoC

    """
    def __init__(self):
        super().__init__('state_machine')

        # ROS2 Parameters
        self.declare_parameter('us_thresh', 0.1)
        self.declare_parameter('rot_vel', 0.3)
        self.declare_parameter('lin_vel', 1.0)

        # Important ROS objects
        self.timer = self.create_timer(0.1, lambda: self.state())  # the single dirtiest hack I've ever written
        self.state = self.rotate  # Define initial state

        self.command_pub = self.create_publisher(SerialCommand, 'command_send', 10)
        #self.pose_sub = self.create_subscription(RobotPose, 'pose_est', self.pose_callback, 10)
        self.marble_sub = self.create_subscription(MarbleArray, 'marbles', self.marble_callback, 10)
        self.us_sub = self.create_subscription(UltrasonicDistances, 'ultrasonic_distances', self.us_callback, 10)

        # Observation
        self.x, self.y, self.th = 0, 0, 0
        self.marbles = []
        self.closest = None
        self.closest_d2 = math.inf
        self.us_left, self.us_right = math.inf, math.inf

        # State variables
        self.linear_time = -math.inf
        self.rot_dir = 1
        self.rotation_time = time.time()

    #def pose_callback(self, msg): self.x, self.y, self.th = msg.x, msg.y, msg.th
    def us_callback(self, msg): self.us_left, self.us_right = msg.left, msg.right
    def marble_callback(self, msg): 
        self.closest, self.closest_d2 = None, math.inf
        for m in msg.data:
            md2 = m.x**2 + m.z**2
            if md2 < self.closest_d2:
                self.closest, self.closest_d2 = m, md2

    
    def rotate(self):
        self.get_logger().info('STATE: rotate')
        us_thresh = self.get_parameter('us_thresh').get_parameter_value().double_value
        w = self.get_parameter('rot_vel').get_parameter_value().double_value


        rot_cmd = SerialCommand()
        rot_cmd.id, rot_cmd.p2 = 8, w*self.rot_dir

        self.get_logger().info(' '.join(['Ultrasonics', str(self.us_left), str(self.us_right)]))

        if self.us_left > us_thresh and self.us_right > us_thresh:
            if self.closest is not None:
                self.get_logger().info(' '.join(['closest', str(self.closest.x), str(self.closest.z)]))

                if self.closest.x*self.rot_dir > 0:
                    self.rot_dir *= -1

            if time.time() - self.rotation_time > 5:
                self.linear_time, self.rotation_time = time.time(), -math.inf
                self.state = self.linear

        self.command_pub.publish(rot_cmd)

    
    def linear(self):
        self.get_logger().info('STATE: linear')
        us_thresh = self.get_parameter('us_thresh').get_parameter_value().double_value
        lin_vel = self.get_parameter('lin_vel').get_parameter_value().double_value

        cmd = SerialCommand()
        cmd.id = 8
        
        self.get_logger().info(' '.join(['Ultrasonics', str(self.us_right), str(self.us_left)]))

        if self.us_left < us_thresh or self.us_right < us_thresh or time.time() - self.linear_time > 3.0: 
            self.linear_time, self.rotation_time = -math.inf, time.time()
            self.state = self.rotate

        else: cmd.p1, cmd.p2 = lin_vel, 0.0

        self.command_pub.publish(cmd)
            

def main(args=None):
    rclpy.init(args=args)
    simple_state_machine = SImplestStateMachineNode()
    rclpy.spin(simple_state_machine)
    simple_state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()