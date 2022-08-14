import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial_communication.serial_communication.uart as sc


class SerialCommunicationNode(Node):

    def __init__(self):
        super().__init__('serial_communication')
        self.subscription = self.create_subscription(Twist, 'topic', self.motor_callback, 10)

    def motor_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    serial_communication = SerialCommunicationNode()

    rclpy.spin(serial_communication)

    serial_communication.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()