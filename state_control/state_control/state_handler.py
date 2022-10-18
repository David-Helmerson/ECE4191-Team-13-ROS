from asyncio.log import logger
import math
import random
from turtle import width
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from project_interfaces.msg import SerialCommand, UltraSonicDistances, RobotPose, MarbleArray

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
        self.declare_parameter('rot_vel', 0.2)
        self.declare_parameter('orient_thresh', 0.1)
        self.declare_parameter('obtain_thresh', 0.1)
        self.declare_parameter('explore_ang_thresh', 0.5)
        self.declare_parameter('explore_vel', 0.12)
        self.declare_parameter('explore_dist', 0.3)
        self.declare_parameter('assurance_vel', 0.12)
        self.declare_parameter('assurance_dist', 0.1)
        self.declare_parameter('avoidance_ang', 0.5)

        # Important ROS objects
        self.timer = self.create_timer(0.1, lambda: self.state())  # the single dirtiest hack I've ever written
        self.state = self.rotate_and_observe  # Define initial state

        self.command_pub = self.create_publisher(SerialCommand, 'command_send', 10)
        self.pose_sub = self.create_subscription(RobotPose, 'pose_est', self.pose_callbcak, 10)
        self.marble_sub = self.create_subscription(MarbleArray, 'marbles', self.marble_callback, 10)
        self.us_sub = self.create_subscription(UltraSonicDistances, 'us_dists', self.us_callback, 10)

        # Observation
        self.x, self.y, self.th = 0, 0, 0
        self.marbles = []
        self.us_left, self.us_right = math.inf, math.inf

        # Interstate variables
        self.clear_vars()
    
    def clear_vars(self): 
        self.rot_t, self.rad_dists = None, []
        self.target_x, self.target_y = None, None
        self.explore_angle, self.explore_t = None, None
        self.ao_last_right, self.ao_extra_t = None, None

    def pose_callback(self, msg): self.x, self.y, self.th = msg.x, msg.y, msg.th
    def us_sub(self, msg): self.us_left, self.us_right = msg.left, msg.right
    def marble_callback(self, msg): self.marbles = [[m.x, m.z] for m in msg.data]


    def determine_explore_angle(self):
        # TODO: stress test
        us_thresh = self.get_parameter('us_thresh').get_parameter_value().double_value
        ang_thresh = self.get_parameter('explore_ang_thresh').get_parameter_value().double_value

        # Find ends of vacant space
        dists = np.array(self.rad_dists)
        vacant = np.logical_and(dists[:, 1] > us_thresh, dists[:, 2] > us_thresh)
        ends = np.argwhere(np.diff(vacant)).squeeze()
        ends += np.array([(not vacant[0]) ^ i%2 for i in range(len(ends))])
        # Handle case with no obstruction
        if np.all(vacant): return random.uniform(0, 2*math.pi)

        # Handle wraparound
        if vacant[0]:
            if vacant[-1]: ends = np.roll(ends, 1)
            else: np.insert(ends, 0, 0)
        elif vacant[-1]: np.insert(ends, -1, 0)

        # Determine valid explore ranges
        angs = dists[ends, 0]
        ranges, widths = [], []
        for i in range(0, len(angs), 2):
            a1, a2 = angs[i], angs[i+1]
            if a1 > a2: a1, a1 -= 2*math.pi
            if a2 - a1 > 2*ang_thresh:
                ranges.append((a1 + ang_thresh, a2 - ang_thresh))
                widths.append((a2 - a1 - 2*ang_thresh))

        range = ranges[np.random.choice(len(ranges), p=widths)]
        return random.uniform(range[0], range[1])

        
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
                self.explore_angle = self.determine_explore_angle()
                self.state = self.explore_arena_rot

        # Publish rotation command
        self.command_pub.publish(rot_cmd)


    def explore_arena_rot(self):
        self.get_logger().info('STATE: explore_arena_rot')
        w_thresh = self.get_parameter('orient_thresh').get_parameter_value().double_value
        w = self.get_parameter('rot_vel').get_parameter_value().double_value

        heading = (self.th - self.explore_angle) % (2*math.pi)
        rot_cmd = SerialCommand()
        rot_cmd.id, rot_cmd.p1, rot_cmd.p2, = 8, 0.0, w*np.sign(heading)
        self.command_pub.publish(rot_cmd)

        if abs(heading) < w_thresh:
            self.explore_angle, self.rad_dists = None, None
            self.state = self.explore_angle_lin

    def explore_arena_lin(self):
        self.get_logger().info('STATE: explore_arena_lin')
        us_thresh = self.get_parameter('us_thresh').get_parameter_value().double_value
        v = self.get_parameter('explore_vel').get_parameter_value().double_value
        d = self.get_parameter('explore_dist').get_parameter_value().double_value
        t = self.get_clock().now()

        if self.explore_t is None: self.explore_t = t
        vel_cmd = SerialCommand()
        vel_cmd.id, vel_cmd.p2 = 8, 0.0

        if self.us_left < us_thresh or self.us_right < us_thresh:
            vel_cmd.p1 = 0.0
            self.clear_vars()
            self.state = self.rotate_and_observe

        elif t - self.explore_t < d/v:
            vel_cmd.p1 = v

        else:
            vel_cmd.p1 = 0.0
            self.clear_vars()
            self.state = self.rotate_and_observe

        self.command_pub.publish(vel_cmd)


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
        self.get_logger().info('STATE: avoid_obstacle')
        w = self.get_parameter('rot_vel').get_parameter_value().double_value

        rot_cmd = SerialCommand()
        rot_cmd.id, rot_cmd.p1 = 8, 0.0

        if self.right: 
            rot_cmd.p2 = w
            self.ao_last_right = True
        elif self.left: 
            rot_cmd.p2 = -w
            self.ao_last_right = False
        else:
            self.state = self.avoid_obstacle_extra

        self.command_pub.publish(rot_cmd)

    def avoid_obstacle_extra(self):
        self.get_logger().info('STATE: avoid_obstacle_extra')
        w = self.get_parameter('rot_vel').get_parameter_value().double_value
        th = self.get_parameter('avoidance_ang').get_parameter_value().double_value
        t = self.get_clock().now()

        rot_cmd = SerialCommand()
        rot_cmd.id, rot_cmd.p1, rot_cmd.p2 = 8, 0.0, 0.0

        if self.ao_extra_t is None: self.ao_extra_t = t
        if t-self.ao_extra_t < th/w:
            rot_cmd.p2 = w if self.ao_last_right else -w
        else: 
            self.ao_last_right, self.ao_extra_t = None, None
            self.state = self.avoid_obstacle_lin
        
        self.command_pub.publish(rot_cmd)

    def avoid_obstacle_lin(self):
        self.get_logger().info('STATE: avoid_obstacle_lin')
        us_thresh = self.get_parameter('us_thresh').get_parameter_value().double_value
        v = self.get_parameter('explore_vel').get_parameter_value().double_value
        d = self.get_parameter('explore_dist').get_parameter_value().double_value
        t = self.get_clock().now()

        if self.explore_t is None: self.explore_t = t
        vel_cmd = SerialCommand()
        vel_cmd.id, vel_cmd.p2 = 8, 0.0

        if self.us_left < us_thresh or self.us_right < us_thresh:
            vel_cmd.p1 = 0.0
            self.explore_t = None
            self.state = self.avoid_obstacle

        elif t - self.explore_t < d/v:
            vel_cmd.p1 = v

        else:
            vel_cmd.p1 = 0.0
            self.explore_t = None
            self.state = self.refind_marble

        self.command_pub.publish(vel_cmd)
        

    def refind_marble(self):
        self.get_logger().info('STATE: refind_marble')
        w_thresh = self.get_parameter('orient_thresh').get_parameter_value().double_value
        w = self.get_parameter('rot_vel').get_parameter_value().double_value

        target_th = math.atan2(self.target_x, self.target_y)

        heading = (self.th - target_th) % (2*math.pi)
        rot_cmd = SerialCommand()
        rot_cmd.id, rot_cmd.p1, rot_cmd.p2, = 8, 0.0, w*np.sign(heading)

        if abs(heading) < w_thresh:
            rot_cmd.p2 = 0.0
            if len(self.marbles):
                self.state = self.orient_to_marble
            else:
                self.clear_vars()
                self.state = self.rotate_and_observe

        self.command_pub.publish(rot_cmd)


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
                self.clear_vars()
                self.state = self.assurance_movement

            # Publish waypoint command
            self.command_pub.publish(wp_cmd)


    def assurance_movement(self):
        self.get_logger().info('STATE: assurance_movement')
        v = self.get_parameter('assurance_vel').get_parameter_value().double_value
        d = self.get_parameter('assurance_dist').get_parameter_value().double_value
        t = self.get_clock().now()

        if self.explore_t is None: self.explore_t = t
        vel_cmd = SerialCommand()
        vel_cmd.id, vel_cmd.p1, vel_cmd.p2 = 8, 0.0, 0.0

        if t - self.explore_t < d/v: vel_cmd.p1 = v
        self.command_pub.publish(vel_cmd)

        # Just move forward tbh
        self.clear_vars()
        self.state = self.activate_manipulator


    def activate_manipulator(self):
        self.get_logger().info('STATE: activate_manipulator')
        manip_cmd = SerialCommand()
        manip_cmd.id = 52
        self.command_pub.publish(manip_cmd)
        self.clear_vars()
        self.state = self.rotate_and_observe




def main(args=None):
    rclpy.init(args=args)
    state_machine = StateMachineNode()
    rclpy.spin(state_machine)
    state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()