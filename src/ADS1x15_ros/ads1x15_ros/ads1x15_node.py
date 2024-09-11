import math
import Adafruit_ADS1x15
import std_msgs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from rclpy.executors import SingleThreadedExecutor



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
            FluidPressure,
            "pressure",
            10,
        )
        self.pub_clk_ = self.create_timer(
            1 / self.pub_rate,
            self.publish_cback,
        )

        self.pressure_msg = FluidPressure()
        self.pressure_msg.header.frame_id = self.frame_id
        self.pressure_msg.variance = 0.01
        self.min_pressure = 80000.0
        self.max_pressure = 110000.0
        self.min_voltage = 0.0
        self.max_voltage = 5.0
        self.adc_min_voltage = 0.0
        self.adc_max_voltage = 6.144
        self.adc_min_digital = 0.0
        self.adc_max_digital = 32767.0
        
    def reading_to_voltage(self, reading_value):
        
        voltage = (reading_value - self.adc_min_digital)/(self.adc_max_digital - self.adc_min_digital) * (self.adc_max_voltage - self.adc_min_voltage) + self.adc_min_voltage

        return voltage
    
    def voltage_to_pressure(self, voltage_value):
        
        pressure = (voltage_value - self.min_voltage)/(self.max_voltage - self.min_voltage) * (self.max_pressure - self.min_pressure) + self.min_pressure

        return pressure

    def publish_cback(self):
        stamp = self.get_clock().now().to_msg()
        adc_value = self.adc.get_last_result()
        voltage = self.reading_to_voltage(adc_value)
        pressure = self.voltage_to_pressure(voltage)
        self.pressure_msg.header.stamp = stamp
        self.pressure_msg.fluid_pressure = pressure
        self.setra_pub_.publish(self.pressure_msg)


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
