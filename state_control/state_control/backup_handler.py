import math
import time
import rclpy
from rclpy.node import Node
from project_interfaces.msg import SerialCommand, UltrasonicDistances, RobotPose, MarbleArray

class SimpleStateMachineNode(Node):
    """
    SImpler central decision making node for the robot, based on a finite state machine.

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
        self.declare_parameter('us_thresh', 0.1)  # IS THIS IN cm OR m
        self.declare_parameter('rot_vel', 0.3)
        self.declare_parameter('lin_vel', 1.0)
        self.declare_parameter('orient_thresh', 0.1)
        self.declare_parameter('obtain_thresh', 0.05)
        self.declare_parameter('assurance_dist', 0.1)

        # Important ROS objects
        self.timer = self.create_timer(0.1, lambda: self.state())  # the single dirtiest hack I've ever written
        self.state = self.rotate  # Define initial state

        self.command_pub = self.create_publisher(SerialCommand, 'command_send', 10)
        self.pose_sub = self.create_subscription(RobotPose, 'pose_est', self.pose_callback, 10)
        self.marble_sub = self.create_subscription(MarbleArray, 'marbles', self.marble_callback, 10)
        self.us_sub = self.create_subscription(UltrasonicDistances, 'ultrasonic_distances', self.us_callback, 10)

        # Observation
        self.x, self.y, self.th = 0, 0, 0
        self.marbles = []
        self.closest = None
        self.closest_d2 = math.inf
        self.us_left, self.us_right = math.inf, math.inf

        # State variables
        self.obtain_time = None
        self.obtain_x, self.obtain_y = None, None
        self.rot_dir = 1  # 1 for counterclockwise, -1 for clockwise

    def pose_callback(self, msg): self.x, self.y, self.th = msg.x, msg.y, msg.th
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

        self.get_logger().info(' '.join([str(self.us_left), str(self.us_right)]))
        if self.us_left < us_thresh or self.us_right < us_thresh:
            rot_cmd.p2 = 0.0
            self.state = self.avoid_obstacle

        elif self.closest is not None:
            self.get_logger().info(' '.join([str(self.closest.x), str(self.closest.z)]))
            rot_cmd.p2 = 0.0
            self.state = self.rotate_to_marble

        self.command_pub.publish(rot_cmd)

    
    def avoid_obstacle(self):
        self.get_logger().info('STATE: avoid_obstacle')
        us_thresh = self.get_parameter('us_thresh').get_parameter_value().double_value
        w = self.get_parameter('rot_vel').get_parameter_value().double_value

        rot_cmd = SerialCommand()
        rot_cmd.id, rot_cmd.p2 = 8, w

        if self.us_left < us_thresh:
            self.rot_dir = 1
            rot_cmd.p2 = -w

        elif self.us_right < us_thresh:
            self.rot_dir = -1

        else:
            rot_cmd.p2 = 0.0
            self.state = self.rotate

        self.command_pub.publish(rot_cmd)


    def rotate_to_marble(self):
        self.get_logger().info('STATE: rotate_to_marble')
        us_thresh = self.get_parameter('us_thresh').get_parameter_value().double_value

        cmd = SerialCommand()
        cmd.id = 8
        
        if self.us_left < us_thresh or self.us_right < us_thresh: self.state = self.avoid_obstacle
        elif self.closest_d2 is None or self.closest is None: self.state = self.rotate
        else:
            orient_thresh = self.get_parameter('orient_thresh').get_parameter_value().double_value
            ang = math.atan(self.closest.x/self.closest.z)

            if ang < orient_thresh: 
                d = self.get_parameter('assurance_dist').get_parameter_value().double_value + math.sqrt(self.closest_d2)
                self.obtain_x, self.obtain_y = self.x - d*math.sin(ang+self.th), self.y + d*math.cos(ang+self.th)
                self.obtain_time = time.time()
                self.state = self.obtain_marble
            else: cmd.id, cmd.p1 = 61, ang

        self.command_pub.publish(cmd)

    
    def obtain_marble(self):
        self.get_logger().info('STATE: obtain_marble')
        us_thresh = self.get_parameter('us_thresh').get_parameter_value().double_value

        cmd = SerialCommand()
        cmd.id = 8
        
        self.get_logger().info(' '.join(['Ultrasonics', str(self.us_right), str(self.us_left)]))
        if self.us_left < us_thresh or self.us_right < us_thresh: 
            self.obtain_x, self.obtain_y = None, None
            self.obtain_time = None
            self.state = self.avoid_obstacle

        elif self.obtain_x is None or self.obtain_time is None:  # Catch unexpected behaviour
            self.state = self.rotate

        elif (time.time() - self.obtain_time) > 3.0:
            self.state = self.rotate

        else:
            cmd.id = 8
            obtain_thresh = self.get_parameter('obtain_thresh').get_parameter_value().double_value**2
            
            dx, dy = self.obtain_x-self.x, self.obtain_y - self.y
            s, c = math.cos(self.th), math.sin(self.th)
            x, y = dx*c - dy*s, dx*s + dy*c
            self.get_logger().info(' '.join(['Moving to', str(x), str(y)]))
            cmd.p1, cmd.p2 = 0.1, 0.0

            if x**2 + y**2 < obtain_thresh: 
                self.obtain_x, self.obtain_y = None, None
                self.obtain_time = None
                self.state = self.rotate

        self.command_pub.publish(cmd)
            

def main(args=None):
    rclpy.init(args=args)
    simple_state_machine = SimpleStateMachineNode()
    rclpy.spin(simple_state_machine)
    simple_state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()