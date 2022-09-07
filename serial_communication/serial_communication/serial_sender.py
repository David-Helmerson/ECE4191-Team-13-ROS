import rclpy
import serial
from rclpy.node import Node
import struct
from project_interfaces.msg import SerialCommand
from std_msgs.msg import UInt8

class SerialSenderNode(Node):
    """
    Writes to serial whenever a message is recieved from the command topic
    ...

    Parameters
    ----------
    port_number: string
        id of the serial port to be read from
    baud_rate: int
        baud rate of the specified serial port

    Topics
    ------
    command_send: L{project_interfaces.SerialCommand} message
        Topic for recieving commands to send
    serial_resync: L{std_msgs.UInt8} message
        Number of dummy bytes to send when PSoC is out of sync

    """
    def __init__(self):
        super().__init__('serial_sender')

        self.start_bytes = b'\xff\xff\xff'
        self.end_bytes = b'\x55\x55\x55'
        
        # ROS2 parameters
        self.declare_parameter('port_number', '/dev/ttyAMA2')
        self.declare_parameter('baud_rate', 57600)
        port_num = self.get_parameter('port_number').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Important objects
        print(port_num, baud_rate)
        self.serial = serial.Serial(port_num, baud_rate)
        self.sub = self.create_subscription(SerialCommand, 'command_send', self.command_callback, 10)
        self.resync_sub = self.create_subscription(UInt8, 'serial_resync', self.sync_callback, 10)

    def command_callback(self, msg):
        ser = self.start_bytes + struct.pack('<Bff', msg.id, msg.p1, msg.p2) + self.end_bytes
        self.get_logger().info(str(ser))
        self.serial.write(ser)

    def sync_callback(self, msg):
        print('resyncing', msg.data)
        self.serial.write(b'\x00'*msg.data)


def main(args=None):
    rclpy.init(args=args)
    serial_sender = SerialSenderNode()
    rclpy.spin(serial_sender)
    serial_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()