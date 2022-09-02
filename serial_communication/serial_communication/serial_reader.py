import rclpy
import serial
from rclpy.node import Node
import struct
from std_msgs.msg import UInt8
from project_interfaces.msg import SerialCommand

class SerialReaderNode(Node):
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

        self.start_bytes = b'\xff\xff\xff'
        self.end_bytes = b'\x55\x55\x55'

        # ROS2 parameters
        self.declare_parameter('port_number', '/dev/ttyAMA2')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('frequency', 100.0)
        port_num = self.get_parameter('port_number').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        timer_freq = self.get_parameter('frequency').get_parameter_value().double_value

        # Important objects
        self.timer = self.create_timer(1/timer_freq, self.timer_callback)
        self.serial = serial.Serial(port_num, baud_rate)

        # TODO: Create publishers for each command
        self.command_pub = self.create_publisher(SerialCommand, 'command_send', 10)
        self.resync_pub = self.create_publisher(UInt8, 'serial_resync', 10)

    def timer_callback(self):
        # Publish update whenever there is data in the buffer
        if self.serial.inWaiting() >= 15:
            print('recieved', self.serial.inWaiting(), 'bytes')
            in_bytes = self.serial.read(15)

            # Resynchronization
            loop_index = in_bytes.find(b'\x55\xff')
            if loop_index >= 0:
                print('Resyncing bytes:', loop_index)
                msg = SerialCommand()
                msg.id, msg.p1 = 90, float(14-loop_index)
                self.command_pub.publish(msg)

            else:
                test1, id, p1, p2, test2 = struct.unpack('>3sBff3s', in_bytes)
                print('data:', test1, id, p1, p2, test2)

                # TODO: Publish p1, p2 to relevant topics
                if id == 90: 
                    msg = UInt8()
                    msg.data = int(p1)
                    self.resync_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    serial_reader = SerialReaderNode()
    rclpy.spin(serial_reader)
    serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()