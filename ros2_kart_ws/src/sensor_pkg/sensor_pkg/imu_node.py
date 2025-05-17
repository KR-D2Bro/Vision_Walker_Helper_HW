#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import Imu
from smbus2 import SMBus
import struct
import math
import time

MPU9250_ADDR      = 0x68
PWR_MGMT_1        = 0x6B
GYRO_CONFIG       = 0x1B
GYRO_XOUT_H       = 0x43
ACCEL_CONFIG      = 0x1C

# 자이로 축 스케일 (±250°/s)
GYRO_SCALE_MODIFIER_250DEG = 131.0

class MPU9250GyroPublisher(Node):
    def __init__(self):
        super().__init__('mpu9250_gyro_publisher')
        
        # 센서 초기화
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(2, self.timer_callback)  # 50Hz
        self.init_bus()
        self.get_logger().info('MPU9250 Gyro publisher started, publishing on /imu/data')

    def read_word_2c(self, reg_high):
        # 두 바이트(High, Low) 읽어서 signed 16-bit로 변환
        high = self.bus.read_byte_data(MPU9250_ADDR, reg_high)
        low  = self.bus.read_byte_data(MPU9250_ADDR, reg_high+1)
        val  = (high << 8) | low
        if val & 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def init_bus(self):
        try:
            self.bus.close()
        except Exception:
            pass

        while True:
            try:
                self.bus = SMBus(1)  # Raspberry Pi I2C bus 1

                self.bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)  # sleep 해제
                self.bus.write_byte_data(MPU9250_ADDR, GYRO_CONFIG, 0x00)  # ±250°/s 회전 감도
                self.bus.write_byte_data(MPU9250_ADDR, ACCEL_CONFIG, 0x00) #가속도 감도
                self.get_logger().info('I2C 초기화 성공')
                break

            except OSError as e:
                self.get_logger().warn(f'I2C 초기화/설정 실패: {e!r}, 5초 후 재시도...')
                time.sleep(5)


    def timer_callback(self):
        try:
            # 자이로 원시값 읽기
            gx = self.read_word_2c(GYRO_XOUT_H)
            gy = self.read_word_2c(GYRO_XOUT_H+2)
            gz = self.read_word_2c(GYRO_XOUT_H+4)

            # 센서 스케일 적용 (raw → rad/s)
            gx = math.radians(gx / GYRO_SCALE_MODIFIER_250DEG)
            gy = math.radians(gy / GYRO_SCALE_MODIFIER_250DEG)
            gz = math.radians(gz / GYRO_SCALE_MODIFIER_250DEG)
        except OSError as e:
            self.get_logger().warn(f'I2C 통신 오류: {e!r}, 버스 재초기화 시도')
            self.init_bus()
            return

        imu_msg = Imu()
        now = self.get_clock().now().to_msg()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu_link'

        # 자이로만 채우기 (가속도·자세는 0으로)
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        # covariance 미지정 상태
        imu_msg.angular_velocity_covariance[0] = -1.0

        self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU9250GyroPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.bus.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

