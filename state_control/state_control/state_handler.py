from asyncio.log import logger
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from project_interfaces.msg import SerialCommand, UltraSonicDistances, Waypoint, RobotPose, RobotVelocity

class StateMachineNode(Node):
    """
    Central decision making node for the robot, based on a finite state machine.

    States
    ------
    rotate_and_observe:
        Rotate until a marble is found, and record U.S. distances
    
    explore_arena:
        Determine unobstructed direction, and move a fixed distance.

    orient_to_marble:
        Rotate until a marble is center screen, or if marble is lost or obstacle is obstructing

    move_to_marble:
        Send waypoints of marble position to PSoC, until close enough or obstrcuted/lost

    find_marble:
        Orient towards the last seen marble, and determine if visible

    assurance_movement:
        Predefined movement sequence to magnetize marble

    activate_manipulator:


    ...

    Parameters
    ----------
    add_out : int
        Integer added to the input when published to output topic

    Topics
    ------
    example_input : L{std_msgs.Int8}
        Subscribed integer input

    example_output : L{std_msgs.Int8}
        Published output upon recieving input, equal to example_input + add_out

    """
    def __init__(self):
        super().__init__('state_machine')

        # ROS2 Parameters
        self.declare_parameter('us_thresh', 10.0)
        self.declare_parameter('rot_vel', 0.4)
        self.declare_parameter('orient_thresh', 0.1)
        self.declare_parameter('obtain_thresh', 0.1)

        # Important ROS objects
        self.timer = self.create_timer(0.1, lambda: self.state())  # the single dirtiest hack I've ever written
        self.state = self.rotate_and_observe  # Define initial state

        self.command_pub = self.create_publisher(SerialCommand, 'command_send', 10)
        self.pose_sub = self.create_subscription(RobotPose, 'pose_est', self.pose_callbcak, 10)

        # Observation
        self.x, self.y, self.th = 0, 0, 0
        self.marbles = []
        self.us_left, self.us_right = math.inf, math.inf

        # Interstate variables
        self.rot_t, self.rad_dists = None, []
        self.target_x, self.target_y = None, None


    def clear_vars(self): 
        self.rot_t, self.rad_dists = None, []
        self.target_x, self.target_y = None, None
    def pose_callback(self, msg): self.x, self.y, self.th = msg.x, msg.y, msg.th

        
    def rotate_and_observe(self):
        self.get_logger().info('STATE: rotate_and_observe')
        w = self.get_parameter('rot_vel').get_parameter_value().double_value
        t = self.get_clock().now()

        # Create rotation command
        rot_cmd = SerialCommand()
        rot_cmd.id, rot_cmd.p1, rot_cmd.p2 = 13, 0.0, w 

        # Marble observed
        if len(self.marbles) > 0:
            self.clear_vars()
            rot_cmd.p2 = 0.0
            self.state = self.orient_to_marble

        else:
            self.rad_dists.append([self.th, self.us_left, self.us_right])

            if self.rot_t is None: # Initialize rotation
                self.rot_t = t 

            elif t > 2*math.pi/w: # Complete rotation
                self.rot_t = None
                rot_cmd.p2 = 0.0
                self.state = self.explore_arena

        # Publish rotation command
        self.command_pub.publish(rot_cmd)


    def explore_arena(self):
        # TODO
        self.get_logger().info('STATE: explore_arena')


    def orient_to_marble(self):
        self.get_logger().info('STATE: orient_to_marble')
        us_thresh = self.get_parameter('us_thresh').get_parameter_value().double_value
        w = self.get_parameter('rot_vel').get_parameter_value().double_value
        w_thresh = self.get_parameter('orient_thresh').get_parameter_value().double_value

        # Create rotation command
        rot_cmd = SerialCommand()
        rot_cmd.id, rot_cmd.p1, rot_cmd.p2, = 8, 0.0, 0.0

        # Obstacle in the way
        if self.us_left < us_thresh or self.us_right < us_thresh:
            self.state = self.avoid_obstacle

        # Marble lost
        elif len(self.marbles) == 0:
            self.state = self.refind_marble

        else:
            # Find closest marble
            marbles = np.array(self.marbles)
            closest = marbles[marbles[:, 1].argmin()]
            self.target_x = self.x + math.cos(self.th)*closest[1] + math.sin(self.th)*closest[0]
            self.target_y = self.y + math.sin(self.th)*closest[1] - math.cos(self.th)*closest[0]

            # Rotate towards closest marble
            if math.atan(abs(closest[0])/closest[1]) > w_thresh:
                rot_cmd.p2 = -np.sign(closest[0])*w
                
            # Stop rotating when close enough
            else:
                rot_cmd.p2 = 0.0
                self.state = self.move_towards_marble
            
        # Publish rotation command
        self.command_pub.publish(rot_cmd)

    
    def avoid_obstacle(self):
        # TODO
        self.get_logger().info('STATE: avoid_obstacle')

    def refind_marble(self):
        # TODO
        self.get_logger().info('STATE: refind_marble')


    def move_towards_marble(self):
        self.get_logger().info('STATE: move_towards_marble')
        us_thresh = self.get_parameter('us_thresh').get_parameter_value().double_value
        w_thresh = self.get_parameter('orient_thresh').get_parameter_value().double_value
        obtain_thresh = self.get_parameter('obtain_thresh').get_parameter_value().double_value

        # Obstacle in the way
        if self.us_left < us_thresh or self.us_right < us_thresh:
            self.state = self.avoid_obstacle

        # Marble lost
        elif len(self.marbles) == 0:
            self.state = self.refind_marble

        else:
            # Create waypoint command
            wp_cmd = SerialCommand()
            wp_cmd.id = 0

            # Find closest marble
            marbles = np.array(self.marbles)
            closest = marbles[marbles[:, 1].argmin()]
            self.target_x = self.x + math.cos(self.th)*closest[1] + math.sin(self.th)*closest[0]
            self.target_y = self.y + math.sin(self.th)*closest[1] - math.cos(self.th)*closest[0]

            wp_cmd.p1, wp_cmd.p2 = closest[1], closest[0]

            # Marble is unaligned
            if math.atan(abs(closest[0])/closest[1]) > w_thresh:
                self.state = self.orient_to_marble

            # Marble is close
            elif closest[1] < obtain_thresh:
                self.state = self.assurance_movement

            # Publish waypoint command
            self.command_pub.publish(wp_cmd)


    def assurance_movement(self):
        # TODO
        self.get_logger().info('STATE: assurance_movement')
        





def main(args=None):
    rclpy.init(args=args)
    state_machine = StateMachineNode()
    rclpy.spin(state_machine)
    state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()