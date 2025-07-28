# light_server.py
import rclpy
from rclpy.node import Node
from interfaces.srv import LightCommand
import RPi.GPIO as GPIO

LED_PIN1 = 16 #BCM 번호 기준
LED_PIN2 = 19 #BCM 번호 기준

class LightService(Node):
    def __init__(self):
        super().__init__('light_service_node')

        # GPIO 초기 설정
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LED_PIN1, GPIO.OUT)
        GPIO.setup(LED_PIN2, GPIO.OUT)
        GPIO.output(LED_PIN1, GPIO.LOW)
        GPIO.output(LED_PIN2, GPIO.LOW)

        self.srv = self.create_service(LightCommand, 'light/control', self.handle_light_command)
        self.led_state = False  # False: 꺼짐, True: 켜짐

    def handle_light_command(self, request, response):
        if request.turn_on:
            self.get_logger().info('💡 불을 켭니다.')
            self.light_on()
        else:
            self.get_logger().info('💤 불을 끕니다.')
            self.light_off()


        # 실제 LED 핀 제어 코드는 여기에 작성 가능 (예: GPIO.output)
        response.success = True
        return response

    def light_on(self):
        GPIO.output(LED_PIN1, GPIO.HIGH)
        GPIO.output(LED_PIN2, GPIO.HIGH)
        self.led_state = True

    def light_off(self):
        GPIO.output(LED_PIN1, GPIO.LOW)
        GPIO.output(LED_PIN2, GPIO.LOW)
        self.led_state = False

def main(args=None):
    rclpy.init(args=args)
    node = LightService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

