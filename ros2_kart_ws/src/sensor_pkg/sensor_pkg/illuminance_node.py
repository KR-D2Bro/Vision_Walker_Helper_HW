import rclpy
from rclpy.node import Node
from interfaces.msg import Illuminance
import time

class LightDataPublisher(Node):
    def __init__(self):
        super().__init__('Light_data_publisher')
        
        self.get_logger().info("Light Data Publisher started, publishing on /illuminance/data')


