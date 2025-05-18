#!/usr/bin/env python3
"""
ROS2 Node for BLE GATT Server using Bluezero on Raspberry Pi
"""
import rclpy
from rclpy.node import Node
import threading
import subprocess
import asyncio
#import re
from bluezero import adapter
from bluezero import peripheral
from bluezero import async_tools
from app_interface_pkg.ble_agent import register_agent

class BLEGATTServer(Node):
    def __init__(self):
        super().__init__('ble_gatt_server')

        # Discover the Bluetooth adapter address
        # Get the first available Bluetooth adapter
        adapters = adapter.list_adapters()
        if not adapters:
            self.get_logger().error('No Bluetooth adapters found')
            raise RuntimeError('No Bluetooth adapter found')
        adapter_address = adapters[0]
        self.get_logger().info(f'Using adapter: {adapter_address}')

        # Configure Bluezero Peripheral
        self.ble_periph = peripheral.Peripheral(adapter_address=adapter_address,
                                           local_name='ROS2Pi',
                                           appearance=0
                                           )
        # Assign connection event callbacks
        self.ble_periph.on_connect = self.handle_connect
        self.ble_periph.on_disconnect = self.handle_disconnect

        # Define a custom service and characteristic UUIDs
        SERVICE_UUID = '12345678-1234-5678-1234-56789abcdef0'
        CHAR_UUID    = 'abcdef01-1234-5678-1234-56789abcdef0'

        # Create service
        self.ble_periph.add_service(srv_id=1,
                                    uuid=SERVICE_UUID,
                                    primary=True)

        # Create characteristic with read and write properties
        self.ble_periph.add_characteristic(srv_id=1,
                                           chr_id=1,
                                           uuid=CHAR_UUID,
                                           value=[],
                                           notifying=False,
                                           flags=['encrypt-read', 'encrypt-write','notify'],
                                           read_callback=self.handle_read,
                                           write_callback=self.handle_write)

        # Run the Bluezero peripheral in a background thread
        thread = threading.Thread(target=self.ble_periph.publish)
        thread.daemon = True
        thread.start()

        self.get_logger().info('BLE GATT Server started')

    
    def handle_connect(self, device_address):
        """
        Called when a Central device connects to this Peripheral
        :param device_address: the BD_ADDR of the connecting Central
        """
        self.get_logger().info(f'Central connected: {device_address}')

    def handle_disconnect(self, device_address):
        """
        Called when a Central device disconnects
        :param device_address: the BD_ADDR of the disconnecting Central
        """
        self.get_logger().info(f'Central disconnected: {device_address}')


    def handle_read(self):
        """
        Read callback: returns current value of the characteristic
        """
        self.get_logger().info('Received read request')
        # Return a list of bytes; here just returns a single byte value 0
        return [0]

    def handle_write(self, value):
        """
        Write callback: receives a list of byte values
        """
        self.get_logger().info(f'Received write request: {value}')
        # Implement handling of incoming data as needed

    def notify_callback(notifying, characteristic):
        if notifying:
            async_tools.add_timer_seconds(2, update_value, characteristic)

    def destroy_node(self):
        # Stop Bluezero peripheral
        try:
            self.ble_periph.quit()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    # 1) asyncio 이벤트 루프 가져오기
    loop = asyncio.get_event_loop()

    # 2) 에이전트 등록 (bus 인스턴스도 같이 얻긴 하지만, 버리지 않으면 됨)
    loop.run_until_complete(register_agent())

    # 3) 루프를 백그라운드 스레드에서 계속 돌린다
    loop_thread = threading.Thread(target=loop.run_forever, daemon=True)
    loop_thread.start()

    rclpy.init(args=args)
    node = BLEGATTServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

        loop.call_soon_threadsafe(loop.stop)
        loop_thread.join()

        loop.close()

if __name__ == '__main__':
    main()

