import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist


class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.sub = self.create_subscription(Twist, 'drive', self.drive_callback, 10)

    def drive_callback(self, msg):
        self.get_logger().info('Driver node heard twist')


def main(args=None):
    rclpy.init(args=args)

    motor_driver = MotorDriverNode()

    rclpy.spin(motor_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()