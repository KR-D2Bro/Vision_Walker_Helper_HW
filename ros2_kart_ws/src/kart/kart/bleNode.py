#!/usr/bin/env python3

import sys
import signal
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gatt

# BLE Constants
ADAPTER_NAME = 'hci0'
SERVICE_UUID = '12345678-1234-5678-1234-56789abcde00'
CHAR_UUID    = '12345678-1234-5678-1234-56789abcde01'

class CartCharacteristic(gatt.Characteristic):
    """
    GATT Characteristic handling config and location_update messages.
    Supports read, write, notify.
    """
    def __init__(self, service):
        # service: parent Service instance
        # uuid: CHAR_UUID, flags: ['read','write','notify']
        super().__init__(service, CHAR_UUID, ['read', 'write', 'notify'])
        self.service = service
        self.value_bytes = bytearray()

    def on_read_value(self, offset):
        return list(self.value_bytes)

    def on_write_value(self, value, options):
        data = bytes(value)
        text = data.decode('utf-8', errors='ignore')
        try:
            msg = json.loads(text)
            msg_type = msg.get('type')
            if msg_type == 'config':
                payload = msg.get('payload', {})
                ros_msg = String(); ros_msg.data = json.dumps(payload)
                self.service.node.config_pub.publish(ros_msg)
                self.service.node.get_logger().info(f'[ROS2] cart_config published: {payload}')
                # Send ACK
                ack = {'type':'config_ack','status':'OK'}
                self.value_bytes = bytearray(json.dumps(ack).encode())
                self.notify()
            else:
                self.service.node.get_logger().warn(f'Unknown msg type: {msg_type}')
        except Exception as e:
            self.service.node.get_logger().error(f'Write parse error: {e}')

    def on_start_notify(self):
        self.service.node.get_logger().info('[GATT] Notify subscription started')

    def on_stop_notify(self):
        self.service.node.get_logger().info('[GATT] Notify subscription stopped')

class CartService(gatt.Service):
    """
    GATT Service wrapping the CartCharacteristic.
    """
    def __init__(self, manager, node):
        # manager: DeviceManager instance
        super().__init__(manager, SERVICE_UUID, True)
        self.node = node
        # create and add characteristic
        char = CartCharacteristic(self)
        self.add_characteristic(char)
        self.characteristic = char

class BLEPeripheralManager(gatt.DeviceManager):
    """
    Manages BLE advertising and services.
    """
    def __init__(self, adapter_name, node):
        super().__init__(adapter_name=adapter_name)
        self.service = CartService(self, node)
        self.register_service(self.service)

    def run(self):
        self.start_advertising()
        super().run()

class BLEPeripheralNode(Node):
    def __init__(self):
        super().__init__('ble_peripheral_node')
        # ROS2 pub/sub
        self.config_pub = self.create_publisher(String, 'cart_config', 10)
        self.create_subscription(String, 'location_update', self.on_location_update, 10)
        # BLE
        self.manager = BLEPeripheralManager(ADAPTER_NAME, self)
        threading.Thread(target=self.manager.run, daemon=True).start()
        # shutdown
        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)

    def on_location_update(self, msg: String):
        notification = {'type':'location_update','payload':msg.data}
        char = self.manager.service.characteristic
        char.value_bytes = bytearray(json.dumps(notification).encode())
        char.notify()
        self.get_logger().info(f'[GATT] Sent location_update: {msg.data}')

    def shutdown(self, signum, frame):
        self.get_logger().info('Shutting down BLE peripheral...')
        try: self.manager.stop()
        except: pass
        rclpy.shutdown(); sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = BLEPeripheralNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.get_logger().info('Stopping peripheral...')
        try: node.manager.stop()
        except: pass
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()

