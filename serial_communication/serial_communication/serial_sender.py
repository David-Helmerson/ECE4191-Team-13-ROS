import rclpy
import serial
from rclpy.node import Node
import struct
from project_interfaces.msg import SerialCommand

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

    """
    def __init__(self):
        super().__init__('serial_sender')
        
        # ROS2 parameters
        self.declare_parameter('port_number', '/dev/ttyAMA2')
        self.declare_parameter('baud_rate', 57600)
        port_num = self.get_parameter('port_number').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Important objects
        print(port_num, baud_rate)
        self.serial = serial.Serial(port_num, baud_rate)
        self.sub = self.create_subscription(SerialCommand, 'command_send', self.command_callback, 10)

    def command_callback(self, msg):
        send_bytes = struct.pack('<xBff', msg.id, msg.p1, msg.p2)
        #print(send_bytes)
        self.serial.write(send_bytes)


def main(args=None):
    rclpy.init(args=args)
    serial_sender = SerialSenderNode()
    rclpy.spin(serial_sender)
    serial_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()