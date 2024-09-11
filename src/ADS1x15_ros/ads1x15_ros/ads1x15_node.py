import math
import Adafruit_ADS1x15
import std_msgs
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32


class ADS1x15Node(Node):
    def __init__(self):
        super().__init__("ads1x15_node")

        # Logger
        self.logger = self.get_logger()

        # Parameters
        self.pub_rate = self.declare_parameter("pub_rate", 50).value
        self.frame_id = self.declare_parameter('frame_id', "setra").value

        # ADC instance
        self.adc = Adafruit_ADS1x15.ADS1115(busnum=7)
        self.GAIN = 2/3
        self.adc.start_adc(0, gain=self.GAIN)
        # Publishers
        self.setra_pub_ = self.create_publisher(
            Float32,
            "range",
            10,
        )
        self.pub_clk_ = self.create_timer(
            1 / self.pub_rate,
            self.publish_cback,
        )

        self.float_msg = Float32()
        self.min_voltage = 0.0
        self.max_voltage = 3.3
        self.adc_min_voltage = 0.0
        self.adc_max_voltage = 6.144
        self.adc_min_digital = 0.0
        self.adc_max_digital = 32767.0
        
    def reading_to_voltage(self, reading_value):
        
        voltage = (reading_value - self.adc_min_digital)/(self.adc_max_digital - self.adc_min_digital) * (self.adc_max_voltage - self.adc_min_voltage) + self.adc_min_voltage

        return voltage
    
    def publish_cback(self):
        stamp = self.get_clock().now().to_msg()
        adc_value = self.adc.get_last_result()
        voltage = self.reading_to_voltage(adc_value)
        self.float_msg.data = voltage
        self.setra_pub_.publish(self.float_msg)


def main(args=None):
    rclpy.init(args=args)
    ads1x15_node = ADS1x15Node()

    executor = SingleThreadedExecutor()
    executor.add_node(ads1x15_node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        print("\n>> Received ctrl-c ... bye\n")

    finally:
        executor.shutdown()
        ads1x15_node.destroy_node()


if __name__ == "__main__":
    main()
