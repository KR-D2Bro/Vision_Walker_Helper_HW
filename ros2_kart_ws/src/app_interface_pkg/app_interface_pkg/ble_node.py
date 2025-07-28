#!/usr/bin/env python3
"""
ROS2 Node for BLE GATT Server using Bluezero on Raspberry Pi
"""
import rclpy
from rclpy.node import Node
from threading import Thread
import subprocess
import asyncio
from interfaces.msg import Config, BatteryState
from interfaces.srv import ConfigSrv
from app_interface_pkg.ble_agent import register_agent
from app_interface_pkg.ble_periph import Ble_peripheral


class BLEGATTServer(Node):
    def __init__(self):
        super().__init__('ble_gatt_server')

        self.config = [False, False,0]
        self.config_cli = self.create_client(ConfigSrv,'/srv/config')
        Thread(target = self.wait_config_srv, daemon=True).start()

        self.periph = None
        
        self.ble_pub = self.create_publisher(Config, '/ble/config', 10)
        self.battery_sub = self.create_subscription(BatteryState,
                                                    '/battery_state/data',
                                                    self.battery_listener_callback,
                                                    10)
        self.last_notified_percentage = 100

        self.ble_lock = self.create_timer(0.01, self._dummy_timer)

    def publish_config(self):
        # BLE 쓰레드에서 이 메소드 호출해서 ROS2 토픽에 뿌리도록 함
        msg = Config()
        msg.light = bool(self.config[0])
        msg.alarm = bool(self.config[1])
        msg.volume = int(self.config[2])
        self.ble_pub.publish(msg)
        
        self.get_logger().info(f"Config Published: {self.config}")


    def battery_listener_callback(self, msg:BatteryState):
        percentage = msg.percentage
        if percentage != self.last_notified_percentage:
            if self.periph is not None:
                self.periph.update_battery_state(percentage)
            self.last_notified_percentage = percentage

    def wait_config_srv(self):
        self.config_cli.wait_for_service(timeout_sec=60.0)
        req = ConfigSrv.Request()
        future = self.config_cli.call_async(req)
        future.add_done_callback(self.handle_config_response)

    def handle_config_response(self,future):
        try:
            res = future.result()
            self.config[0] = res.light
            self.config[1] = res.alarm
            self.config[2] = res.volume
        except Exception as e:
            self.get_logger().error(f'[Error] 서비스 요청 중 오류: {e}')

    # dummy 콜백: 이 타이머는 BLE 쓰레드와 시분할로 실행될 때,
    # 자잘한 처리를 하기 위해 빈 콜백을 두는 것 뿐입니다.
    def _dummy_timer(self):
        pass



def main(args=None):
    rclpy.init(args=args)
    node = BLEGATTServer()
    ble_periph = Ble_peripheral(node=node)
    node.periph = ble_periph

    thread = Thread(target=ble_periph.ble_periph.publish, daemon=True)
    thread.start()
    
    # 1) asyncio 이벤트 루프 가져오기
    loop = asyncio.get_event_loop()

    # 2) 에이전트 등록 (bus 인스턴스도 같이 얻긴 하지만, 버리지 않으면 됨)
    loop.run_until_complete(register_agent(ble_periph))

    # 3) 루프를 백그라운드 스레드에서 계속 돌린다
    loop_thread = Thread(target=loop.run_forever, daemon=True)
    loop_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        thread.join()
        thread.close()

        node.destroy_node()
        rclpy.shutdown()

        loop.call_soon_threadsafe(loop.stop)
        loop_thread.join()

        loop.close()

if __name__ == '__main__':
    main()

