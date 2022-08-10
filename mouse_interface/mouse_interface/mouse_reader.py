import rclpy
from rclpy.node import Node
from project_interfaces.msg import MousePos
import struct



class MousePublisher(Node):
    """
    Publishes raw mouse xy data at a fixed rate, likely will publish xy data as metric units in future. Currently lacks required root privileges

    ...
    
    Publishers
    ---
    mouse_deltas: raw X/Y output of mouse, as project_interfaces.MousePos


    """

    def __init__(self):
        super().__init__('mouse_publisher')
        self.mouse_file = open( "/dev/input/mice", "rb" )
        self.publisher_ = self.create_publisher(MousePos, 'mouse_deltas', 10)
        timer_period = 1/30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = MousePos()

        # Read xy coords of mouse
        buf = self.mouse_file.read(3)
        msg.x, msg.y = struct.unpack( "bb", buf[1:] )
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    mouse_publisher = MousePublisher()

    rclpy.spin(mouse_publisher)
    mouse_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()