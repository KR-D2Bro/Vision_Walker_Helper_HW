#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import Imu
from std_msgs.msg import Float32MultiArray
from smbus2 import SMBus
import time
import math
import numpy as np


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # 1) IMU I2C 설정
        self.init_bus(1)                # I2C 버스 번호 (imu : 3)
        self.addr_imu = 0x69           # MPU-9250 IMU 주소

        self.dt = 0.01 #100Hz
        self.alpha = 0.98 #complementary filter 계수
        self.roll = 0.0
        self.pitch = 0.0
        
        # 2) 센서 초기화
        self._wakeup_imu()
        self._config_imu()
        self._bypass_enable()

        # 3) 낙상 기준 (Roll/Pitch 임계각, 도 단위 → 라디안으로 변환)
        self.declare_parameter('roll_threshold_deg', 65.0)
        self.declare_parameter('pitch_threshold_deg', 65.0)
        roll_thr  = self.get_parameter('roll_threshold_deg').value
        pitch_thr = self.get_parameter('pitch_threshold_deg').value
        self.roll_threshold  = math.radians(roll_thr)
        self.pitch_threshold = math.radians(pitch_thr)

        # 4) 퍼블리셔 (/fall_alert 토픽)
        self.pub_rp = self.create_publisher(Float32MultiArray, '/imu/roll_pitch', 10)
        self.pub_alert = self.create_publisher(Imu, '/imu/alert', 10)

        # 5) 타이머 (100Hz)
        self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info(
            f'FallDetectionNode started: roll_thr={roll_thr}°, pitch_thr={pitch_thr}°'
        )
    
    def init_bus(self,bus_num:int):
        try:
            self.bus.close()
        except Exception:
            pass

        while True:
            try:
                self.bus = SMBus(bus_num)

                self.get_logger().info('I2C-3 초기화 성공')
                break
            except OSError as e:
                self.get_logger().warn(f'I2C 초기화 실패: {e!r}, 5초 후 재시도...')
                time.sleep(5)
        
    # ─── 센서 초기화 ─────────────────────────────────────────────────────────
    def _wakeup_imu(self):
        # PWR_MGMT_1 레지스터(0x6B) → 0x00으로 Sleep 해제
        self.bus.write_byte_data(self.addr_imu, 0x6B, 0x00)
        time.sleep(0.1)

    def _config_imu(self):
        # 자이로 (0x1B) 250 dps, 가속도 (0x1C) ±2g
        self.bus.write_byte_data(self.addr_imu, 0x1B, 0x00)
        self.bus.write_byte_data(self.addr_imu, 0x1C, 0x00)
        time.sleep(0.1)

    def _bypass_enable(self):
        # INT_PIN_CFG 레지스터(0x37)에 바이패스 모드 설정
        self.bus.write_byte_data(self.addr_imu, 0x37, 0x02)
        time.sleep(0.1)

    # ─── I2C 읽기 헬퍼 ────────────────────────────────────────────────────────
    def _read_word(self, addr, reg):
        hi = self.bus.read_byte_data(addr, reg)
        lo = self.bus.read_byte_data(addr, reg+1)
        val = (hi << 8) | lo
        return val - 65536 if (val & 0x8000) else val

    def read_accel(self):
        # 가속도 레지스터: 0x3B, 0x3D, 0x3F (각 2바이트)
        # ±2g → 16384 LSB/g
        ax = self._read_word(self.addr_imu, 0x3B) / 16384.0
        ay = self._read_word(self.addr_imu, 0x3D) / 16384.0
        az = self._read_word(self.addr_imu, 0x3F) / 16384.0
        return np.array([ax, ay, az])

    # ─── 타이머 콜백: 낙상 감지 ────────────────────────────────────────────────
    def timer_cb(self):
        try:
            accel = self.read_accel()

            # Roll = atan2(ay, az)
            roll  = math.atan2(accel[1], accel[2])
            # Pitch = atan2(-ax, sqrt(ay² + az²))
            pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))

            # 낙상 감지: 절대각이 임계치 초과 시 True
            fall = (abs(roll)  > self.roll_threshold) or \
                    (abs(pitch) > self.pitch_threshold)

            
            # 로깅
            if fall:
                msg = Imu()
                # 퍼블리시
                self.pub_alert.publish(msg)
                self.get_logger().warn(
                    f'FALL! Roll={math.degrees(roll):.1f}°, '
                    f'Pitch={math.degrees(pitch):.1f}°'
                )
        except:
            self.get_logger().warn('I2C-1 연결 불량 발생')
            self.init_bus(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

