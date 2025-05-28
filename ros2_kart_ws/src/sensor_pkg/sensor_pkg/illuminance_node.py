#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from smbus2 import SMBus

class LightDataPublisher(Node):
    def __init__(self):
        super().__init__('Light_data_publisher')
        self.pub = self.create_publisher(Float32, 'light_intensity', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # I2C 버스 및 PCF8591(기본 주소 0x48) 초기화
        self.bus = SMBus(0)
        self.addr = 0x48
        self.vref = 3.3             # 참조 전압
        self.control_base = 0x40    # PCF8591: 0100_0000 (자동증가 off, 입력 모드)

        self.get_logger().info(f'Initialized PCF8591 at 0x{self.addr:02X}')

    def read_ain(self, channel: int) -> int:
        if not 0 <= channel <= 3:
            raise ValueError('AIN 채널은 0~3만 지원합니다')
        ctrl = self.control_base | channel
        # 제어 바이트 전송
        self.bus.write_byte(self.addr, ctrl)
        # 더미 리드(첫 번째 읽기는 버릴 것)
        self.bus.read_byte(self.addr)
        # 실제 데이터 읽기
        return self.bus.read_byte(self.addr)

    def timer_callback(self):
        try:
            raw = self.read_ain(0)  # 보드 내장 LDR → AIN0
            voltage = raw / 255.0 * self.vref
            msg = Float32(data=voltage)
            self.pub.publish(msg)
            self.get_logger().info(f'RAW={raw:3d} → {voltage:.3f} V')
        except Exception as e:
            self.get_logger().error(f'I2C read error: {e}')

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LightDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

