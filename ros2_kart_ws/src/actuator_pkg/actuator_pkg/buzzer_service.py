#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from interfaces.srv import BuzzerCommand  # 패키지명에 맞게 수정
import time

BUZZER_PIN = 18  # BCM 기준 GPIO 18번

class BuzzerService(Node):
    def __init__(self):
        super().__init__('buzzer_control_node')
        self.get_logger().info('🔔 PWM Buzzer Control Node Started')

        # GPIO 초기화
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUZZER_PIN, GPIO.OUT)

        # PWM 설정 (초기 주파수: 1000Hz)
        self.pwm = GPIO.PWM(BUZZER_PIN, 1000)
        self.pwm.start(0)  # duty cycle 0%로 무음 시작

        # 서비스 생성
        self.srv = self.create_service(BuzzerCommand, 'buzzer/control', self.buzzer_callback)

    def buzzer_callback(self, request, response):
        dc = max(0, min(100, request.duty_cycle))  # 범위 제한
        self.pwm.ChangeDutyCycle(dc)
        self.get_logger().info(f'🔊 Buzzer ON (Duty: {dc}%) {request.time}Sec')
        time.sleep(request.time)
        self.pwm.ChangeDutyCycle(0)
        
        response.success = True
        return response

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BuzzerService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

