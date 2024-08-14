import rclpy
from rclpy.node import Node
import std_msgs.msg
import sys
import ast

from .base_controller import BaseController


class RoverEsp32(Node):
    def __init__(self):
        super().__init__('rover_esp32')

        self.declare_parameter('uart_dev', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        uart_dev = self.get_parameter('uart_dev').value
        baudrate = self.get_parameter('baudrate').value

        self.base = BaseController(uart_dev, baudrate)
        self.get_logger().info(f'Connected to {uart_dev}')

        # Config esp32
        self.base.set_feedback_flow(False)
        self.base.set_feedback_delay_ms(0)

        # Callbacks
        self.state_pub = self.create_publisher(
            std_msgs.msg.String, 'rover/state', 10)
        self.state_timer = self.create_timer(0.001, self.state_callback)
        self.command_sub = self.create_subscription(
            std_msgs.msg.String, 'rover/command', self.command_callback, 10)

    def state_callback(self):
        try:
            self.base.get_feedback()
            status = self.base.read_feedback()
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return
            
        if status is None:
            return

        status['timestamp'] = str(self.get_clock().now().to_msg())

        msg = std_msgs.msg.String()
        msg.data = str(status)
        self.state_pub.publish(msg)

    def command_callback(self, msg):
        try:
            command = msg.data
            command = ast.literal_eval(command)
            self.get_logger().info(f'Command received: {command}')
            self.base.send_command(command)
        except Exception as e:
            print(f'Error: {e}')


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
