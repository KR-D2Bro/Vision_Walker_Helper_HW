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

        # PWM 설정 (초기 주파수: 100Hz)
        self.pwm = GPIO.PWM(BUZZER_PIN, 800)
        self.pwm.start(0)  # duty cycle 0%로 무음 시작

        # 서비스 생성
        self.srv = self.create_service(BuzzerCommand, 'buzzer/control', self.buzzer_callback)

    def buzzer_callback(self, request, response):
        dc = self.set_volume(request.duty_cycle)  # 범위 제한
        self.detect_alert(request.time, float(dc))
        self.get_logger().info(f'🔊 Buzzer ON (Duty: {dc}%) {request.time}Sec')
        
        response.success = True
        return response

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

    def set_volume(self, vol: int):
        vol = max(0, min(100, vol))
        max_dc = 80.0

        norm = (vol/100.0)
        dc = max_dc * (norm ** 2)

        return dc

    def beep_short(self, freq, duration, duty):
        self.pwm.ChangeFrequency(freq)
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(duration)
        self.pwm.ChangeDutyCycle(0)
        time.sleep(0.01)

    def detect_alert(self,duration ,duty=70):
        self.beep_short(1000, duration, duty)
        time.sleep(0.01)
        self.beep_short(1000, duration, duty)

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

