import rclpy
from rclpy.node import Node
import std_msgs.msg

from .base_controller import BaseController


class RoverEsp32Status(Node):
    def __init__(self, uart_dev_set, baud_set):
        super().__init__('rover_esp32_status')
        self.base = BaseControllerSingleton(uart_dev_set, baud_set)
        self.pub = self.create_publisher(
            std_msgs.msg.String, 'rover_status', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            status = self.base.feedback_data()
        except Exception as e:
            print(f'Error: {e}')
            return

        msg = std_msgs.msg.String()
        msg.data = str(status)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    status_publisher = RoverEsp32Status()

    rclpy.spin(status_publisher)

    status_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
