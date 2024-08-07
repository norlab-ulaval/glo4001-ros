import rclpy
from rclpy.node import Node
import std_msgs.msg
import sys
import yaml

from .base_controller import BaseControllerSingleton


class RoverEsp32(Node):
    def __init__(self):
        super().__init__('rover_esp32')

        self.declare_parameter('uart_dev', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        uart_dev = self.get_parameter('uart_dev').value
        baudrate = self.get_parameter('baudrate').value

        self.base = BaseControllerSingleton(uart_dev, baudrate)
        self.get_logger().info(f'Connected to {uart_dev}')

        self.pub = self.create_publisher(std_msgs.msg.String, 'rover', 10)

        # Config esp32
        self.base.set_feedback_flow(False)
        self.base.set_feedback_delay_ms(0)

        # Callbacks
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        try:
            self.base.get_feedback()
            status = self.base.read_feedback()
        except Exception as e:
            print(f'Error: {e}')
            return

        msg = std_msgs.msg.String()
        msg.data = str(status)
        self.pub.publish(msg)


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    status_publisher = RoverEsp32()

    rclpy.spin(status_publisher)

    status_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
