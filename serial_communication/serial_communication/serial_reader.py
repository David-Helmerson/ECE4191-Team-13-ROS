import rclpy
import serial
from rclpy.node import Node
import struct


class BufferFlagNode(Node):
    """
    Reads off of a serial buffer at a fixed rate, and publishes raw recieved command data to
    topics specified by the command.

    ...

    Parameters
    ----------
    port_number: string
        id of the serial port to be read from
    baud_rate: int
        baud rate of the specified serial port
    frequency: double
        frequency at which the node checks the port for updates

    Topics
    ------
    TODO

    """
    def __init__(self):
        super().__init__('buffer_flag')

        # ROS2 parameters
        self.declare_parameter('port_number', 'COM2')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frequency', 100)
        port_num = self.get_parameter('port_number').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        timer_freq = self.get_parameter('frequency').get_parameter_value().double_value

        # Important objects
        self.timer = self.create_timer(1/timer_freq, self.timer_callback)
        self.serial = serial.Serial(port_num, baud_rate)
        # TODO: Create publishers for each command


    def timer_callback(self):
        # Publish update whenever there is data in the buffer
        if self.serial.inWaiting():
            in_bytes = self.serial.read(9)
            command_id = int.from_bytes(in_bytes[0], 'big')
            p1, p2 = struct.unpack('ff', in_bytes[1:])
            # TODO: Publish p1, p2 to relevant topics


def main(args=None):
    rclpy.init(args=args)
    buffer_flag = BufferFlagNode()
    rclpy.spin(buffer_flag)
    buffer_flag.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()