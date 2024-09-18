import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DummyListener(Node):
    def __init__(self):
        super().__init__('dummy_listener')

        # Declare and get the parameter for the topic name
        self.declare_parameter('topic_name', 'dummy_topic')
        topic_name = self.get_parameter(
            'topic_name').get_parameter_value().string_value

        # Create subscription to the given topic
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        ...


def main(args=None):
    rclpy.init(args=args)
    node = DummyListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
