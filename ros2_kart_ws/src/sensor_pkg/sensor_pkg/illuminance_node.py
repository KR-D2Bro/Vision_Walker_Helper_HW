#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import Illuminance
from smbus2 import SMBus
import time


class LightDataPublisher(Node):
    def __init__(self):
        super().__init__('Light_data_publisher')
        self.pub = self.create_publisher(Illuminance, '/light/alert', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.decide_timer = self.create_timer(10.0, self.decide_timer_callback)

        self.tmp = 0            # 누적 판정 버퍼
        self.dark_thr_lux = 10  # 이 값 이하이면 어둡다고 판단
        self.addr = 0x23        # BH1750 기본 주소 (ADDR=GND/NC)

        # I2C 버스 0 초기화 (핀 27:SDA0, 28:SCL0)
        self.bus = SMBus(0)
        self._init_bh1750()

    def _init_bh1750(self):
        try:
            # Power ON, reset, Continuous High-Res Mode (1 lux resolution, ~120ms)
            self.bus.write_byte(self.addr, 0x01)
            time.sleep(0.01)
            self.bus.write_byte(self.addr, 0x07)
            time.sleep(0.01)
            self.bus.write_byte(self.addr, 0x10)
            self.get_logger().info(f'BH1750 initialized on /dev/i2c-0 addr=0x{self.addr:02X}')
        except Exception as e:
            self.get_logger().error(f'BH1750 init failed: {e}')

    def read_lux(self) -> float:
        self.bus.write_byte(self.addr, 0x10)  # 연속 고해상도 모드
        time.sleep(0.18)  # 변환 완료 대기
        data = self.bus.read_i2c_block_data(self.addr, 0x00, 2)
        raw = (data[0] << 8) | data[1]
        lux = raw / 1.2
        self.get_logger().debug(f'BH1750 raw={raw} bytes={data}')
        return lux

    def timer_callback(self):
        try:
            lux = self.read_lux()
            if lux <= self.dark_thr_lux:
                self.tmp += 1
            else:
                self.tmp -= 1
            # 누적치 폭을 제한해 과도한 누적 방지
            self.tmp = max(-50, min(50, self.tmp))
            self.get_logger().info(f'Illuminance: {lux:.1f} lux, tmp={self.tmp}')
        except Exception as e:
            self.get_logger().error(f'I2C read error: {e}')

    def decide_timer_callback(self):
        # tmp>0 이면 야간으로 판단
        res = self.tmp > 0
        self.tmp = 0

        msg = Illuminance()
        msg.is_night = res
        self.pub.publish(msg)
        self.get_logger().info(f'[INFO] Illuminance publish: isNight={res}')

    def destroy_node(self):
        try:
            self.bus.close()
        except Exception:
            pass
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
