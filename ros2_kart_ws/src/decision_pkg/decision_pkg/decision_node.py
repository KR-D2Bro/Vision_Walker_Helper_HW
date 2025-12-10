# decision_node.py
import rclpy
from rclpy.node import Node
from interfaces.srv import LightCommand, BuzzerCommand, ConfigSrv
from interfaces.msg import Imu, Illuminance, Vision, Config, Fall
import time, json
from threading import Timer

HOMEDIR = '/home/dongjae/Vision_Walker_Helper_HW/'
CONFIG_PATH = HOMEDIR + 'config.conf'

CONFIG_KEY = ['light','alarm','volume']

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        self.config = []
        self.load_config_from_disk()
        self.isNight = False
        self.isRinging = False

        self.config_srv = self.create_service(ConfigSrv, '/srv/config',self.handle_config_request)
        self.light_cli = self.create_client(LightCommand, 'light/control')
        self.buzzer_cli = self.create_client(BuzzerCommand, 'buzzer/control')
        self.fall_pub = self.create_publisher(Fall, '/fall/alert', 10)
        self.create_subscription(Illuminance,
                                 '/light/alert',
                                 self.illuminance_listener_callback,
                                 10)
        self.create_subscription(Fall,
                                 '/imu/alert',
                                 self.imu_listener_callback,
                                 10)
        self.create_subscription(Vision,
                                 '/object_detection/alert',
                                 self.vision_listener_callback,
                                 10)
        self.create_subscription(Config,
                                 '/ble/config',
                                 self.config_listener_callback,
                                 10)

    def send_light_request(self, turn_on: bool):
        req = LightCommand.Request()
        req.turn_on = turn_on
        self.light_cli.call_async(req)

    def send_buzzer_request(self, time:float):
        def refresh():
            self.isRinging = False

        if self.config[1] and not self.isRinging:
            self.isRinging = True
            self.create_timer(time, refresh)
            req = BuzzerCommand.Request()
            req.time = time
            req.duty_cycle = self.config[2]
            self.buzzer_cli.call_async(req)
            self.get_logger().info(f'[INFO] send buzzer request: {time} seconds')

        
    def illuminance_listener_callback(self, msg: Illuminance):
        self.isNight = msg.is_night
        self.send_light_request(msg.is_night and self.config[0])
        self.get_logger().info(f'[INFO] send light request: {self.config[0] and msg.is_night}')
        
    def imu_listener_callback(self, msg: Fall):
        self.send_buzzer_request(3.0)
        fall_msg = Fall()
        fall_msg.is_fall = msg.is_fall
        self.fall_pub.publish(fall_msg)

    def vision_listener_callback(self, msg: Vision):
        self.send_buzzer_request(0.08)

    def config_listener_callback(self, msg: Config):
        self.config[0] = msg.light
        self.config[1] = msg.alarm
        self.config[2] = msg.volume

        self.send_light_request(self.isNight and self.config[0])    #라이트 설정 즉각 반영
        self.save_config_to_disk()  #변경값 저장

    def save_config_to_disk(self):
        config_json = {
                'light' : self.config[0],
                'alarm' : self.config[1],
                'volume' : self.config[2]
            }
        try:
            with open(CONFIG_PATH, 'w') as f:
                json.dump(config_json, f)
        except Exception as e:
            self.get_logger().error(f'[Error] Failed to write config to {CONFIG_PATH}: {e}')

    def load_config_from_disk(self):
        try:
            with open(CONFIG_PATH, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(
                f'[Error] Failed to read or parse JSON from {CONFIG_PATH}: {e}'
            )

        result = []
        for key in CONFIG_KEY:
            val = data.get(key,0)
            result.append(val)

        self.config = result[:]
        self.get_logger().info('Loaded config from disk')

    def handle_config_request(self, request, response):
        response.light = self.config[0]
        response.alarm = self.config[1]
        response.volume = self.config[2]
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

