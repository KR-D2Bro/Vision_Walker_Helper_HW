import rclpy
from rclpy.node import Node
from interfaces.msg import BatteryState
from smbus2 import SMBus
import time

I2C_BUS_NUM = 1
DEVICE_ADDR = 0x17

class BatteryStatePublisher(Node):
    def __init__(self):
        super().__init__('Battery_state_publisher')
        self.pub = self.create_publisher(BatteryState, '/battery_state/data', 10)
        self.timer = self.create_timer(10.0, self.timer_callback)
        self.init_bus(I2C_BUS_NUM)
        self.get_logger().info('Battery State Publisher started, publishing on /battery_state/data')


    def timer_callback(self):
        msg = BatteryState()
        try:
            msg.percentage = self.read_battery()
        except Exception as e:
            self.get_logger().warn(f'I2C 통신 오류: {e!r}, 버스 재초기화 시도')
            self.init_bus(I2C_BUS_NUM)
            return

        self.pub.publish(msg)
        self.get_logger().info(f'Battery: {round(msg.percentage,2)}%')

    def read_battery(self):
        msb = self.bus.read_byte_data(DEVICE_ADDR, 20)
        lsb = self.bus.read_byte_data(DEVICE_ADDR, 19)
        percentage = (msb << 8) | lsb
        return percentage

    def init_bus(self, bus_num : int):
        try:
            self.bus.close()
        except Exception:
            pass

        while True:
            try:
                self.bus = SMBus(bus_num)

                self.get_logger().info('I2C 초기화 성공')
                break

            except OSError as e:
                self.get_logger().warn(f'I2C 초기화 실패: {e!r}, 5초 후 재시도...')
                time.sleep(5)

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

