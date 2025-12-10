#!/usr/bin/env python3
import threading
import subprocess
import asyncio, time
#import re
from bluezero import adapter
from bluezero import peripheral
from bluezero import async_tools
import random
import struct
HOMEDIR = '/home/dongjae/Vision_Walker_Helper_HW/'
BINDINGPATH = HOMEDIR + 'binding.conf'
SECKEYPATH = HOMEDIR + 'secretKey.txt'
LOCALNAME = 'VisionWalkHelper'
# Define a custom service and characteristic UUIDs
BIND_SVC_UUID = '87654321-1234-5678-1234-56789abcdef0'
BIND_CHAR_UUID = 'fedcba01-1234-5678-1234-56789abcdef0'
LIGHT_CONFIG_CHAR_UUID = 'abcdef01-1234-5678-1234-56789abcdef1'
ALARM_CONFIG_CHAR_UUID = 'abcdef01-1234-5678-1234-56789abcdef2'
VOLUME_CONFIG_CHAR_UUID = 'abcdef01-1234-5678-1234-56789abcdef3'
BATTERY_CHAR_UUID = '2A19'
FALL_CHAR_UUID = 'abcdef01-1234-5678-1234-56789abcdef6'
AUTH_TIMEOUT_SEC = 2



class Ble_peripheral():
    def __init__(self, node):
        self.node = node
        self.is_authenticated = False
        self.connected_addr = None
        self.battery_notify_characteristic = None
        self.fall_notify_characteristic = None
        self.last_fall_state = False

        # Discover the Bluetooth adapter address
        # Get the first available Bluetooth adapter
        adapters = adapter.list_adapters()
        if not adapters:
            self.node.get_logger().error('No Bluetooth adapters found')
            raise RuntimeError('No Bluetooth adapter found')
        self.adapter_address = adapters[0]
        self.node.get_logger().info(f'Using adapter: {self.adapter_address}')
        
        self.ble_periph = peripheral.Peripheral(
            adapter_address=self.adapter_address,
            local_name=LOCALNAME,
        )
        self.ble_periph.on_connect = self.handle_connect
        self.ble_periph.on_disconnect = self.handle_disconnect

        self._add_advertising_service()
        self._add_secure_config_service()
        self._add_battery_notify_characteristic() 
        self._add_fall_notify_characteristic()

        self.data_lock = threading.Lock()

    def _add_advertising_service(self):
        # Create service
        self.ble_periph.add_service(
            srv_id=1,
            uuid=BIND_SVC_UUID,
            primary=True
        )

        # Create characteristic with read and write properties
        self.ble_periph.add_characteristic(
                srv_id=1,
                chr_id=1,
                uuid=BIND_CHAR_UUID,
                value=[],
                notifying=False,
                flags=['write'],
                write_callback=self._auth_write_callback
        )
                
        self.node.get_logger().info('BLE GATT Server started')

    def _add_secure_config_service(self):
        self.ble_periph.add_characteristic(
            srv_id=1,
            chr_id=2,
            uuid=LIGHT_CONFIG_CHAR_UUID,
            value=[0],
            notifying=False,
            flags=['read', 'write'],
            read_callback=self.light_read_config,
            write_callback=self.light_write_config
        )
        
        self.ble_periph.add_characteristic(
            srv_id=1,
            chr_id=3,
            uuid=ALARM_CONFIG_CHAR_UUID,
            value=[0],
            notifying=False,
            flags=['read', 'write'],
            read_callback=self.alarm_read_config,
            write_callback=self.alarm_write_config
        )

        self.ble_periph.add_characteristic(
            srv_id=1,
            chr_id=4,
            uuid=VOLUME_CONFIG_CHAR_UUID,
            value=[0],
            notifying=False,
            flags=['read', 'write'],
            read_callback=self.volume_read_config,
            write_callback=self.volume_write_config
        )

        self.node.get_logger().info('Config service added')

    def _add_battery_notify_characteristic(self):
        self.ble_periph.add_characteristic(
            srv_id=1,
            chr_id=5,
            uuid=BATTERY_CHAR_UUID,
            value=[self.node.last_notified_percentage],
            notifying=False,
            flags = ['notify'],
            notify_callback=self.battery_notify_callback
        )

    def _add_fall_notify_characteristic(self):
        self.ble_periph.add_characteristic(
            srv_id=1,
            chr_id=6,
            uuid = FALL_CHAR_UUID,
            value=[self.last_fall_state],
            notifying=False,
            flags = ['notify'],
            notify_callback=self.fall_notify_callback
        )

    def update_fall_state(self, fall_state: bool):
        if self.fall_notify_characteristic is not None:
            try:
                data_to_send = struct.pack("!B", 1 if fall_state else 0)
                self.fall_notify_characteristic.set_value(data_to_send)
            except Exception as e:
                self.node.get_logger().warn(f'-------------------errorerrorerror!!!!!!!!! : {e}')

    #notify 구독시 해당 특성을 저장
    def fall_notify_callback(self,notifying, characteristic):
        if notifying:
            self.node.get_logger().info('-------------------fall_notify subscribed!!!!!!!!!')
            self.fall_notify_characteristic = characteristic
        else:
            self.fall_notify_characteristic = None

    def battery_notify_callback(self,notifying, characteristic):
        if notifying:
            self.battery_notify_characteristic = characteristic
        else:
            self.battery_notify_characteristic = None

    #ble_node에서 호출하는 메소드로 배터리 특성의 value를 변경
    def update_battery_state(self, percentage: int):
        if self.battery_notify_characteristic is not None:
            self.battery_notify_characteristic.set_value(list(percentage.to_bytes(2,byteorder='little',signed=True)))


    def _auth_write_callback(self, value, options):
        #value와 키를 비교하여 authenticate 수행
        # 클라이언트가 보낸 값을 utf-8 문자열로 디코딩
        token = bytes(value).decode('utf-8').strip()

        try:
            with open(SECKEYPATH,'r', encoding='utf-8') as f:
                sec_key = f.read().strip()
                self.node.get_logger().info(f'Read secret key success : {sec_key}')

        except Exception as e:
            self.node.get_logger().error(f'Error!! {e}')
            return

        self.node.get_logger().info(f"{sec_key} : {token}") 
        if sec_key == token:
            self.node.get_logger().info(f'[ble_gatt_server] Authentication success')
            self.is_authenticated = True

    def handle_connect(self, device_address):
        def _delayed_auth_check():
            interval = 0.1  # 0.1초 간격으로 폴링
            elapsed = 0.0

            while elapsed < AUTH_TIMEOUT_SEC:
                # 인증이 되었으면 즉시 정상 흐름을 유지하고 종료
                if self.is_authenticated:
                    # 로그를 남기고 바로 종료
                    self.node.get_logger().info(f"[ble_gatt_server] Authenticated after {elapsed:.1f}s, auth success")
                    return

                time.sleep(interval)
                elapsed += interval
            self.connected_addr.disconnect()
        
        self.connected_addr = device_address
        t = threading.Thread(target=_delayed_auth_check, daemon=True)
        t.start()
        self.node.get_logger().info(f'Central connected: {device_address}, auth check start')

    def handle_disconnect(self, device_address):
        self.connected_addr = None
        self.is_authenticated = False
        self.fall_notify_characteristic=None
        self.battery_notify_characteristic = None
        self.node.get_logger().info(f'Central disconnected: {device_address}')

    def light_read_config(self):
        self.node.get_logger().info(f'light_read: {self.node.config[0]}')
        return [int(self.node.config[0])]

    def alarm_read_config(self):
        self.node.get_logger().info(f'alarm_read: {self.node.config[1]}')
        return [int(self.node.config[1])]
    
    def volume_read_config(self):
        self.node.get_logger().info(f'volume_read: {self.node.config[2]}')
        return [int(self.node.config[2])]

    def light_write_config(self, value,options):
        if self.is_authenticated:
            self.node.get_logger().info(f'light_write:{value[0]}')
            self.node.config[0] = bool(value[0])
            self.node.publish_config()
    
    def alarm_write_config(self, value,options):
        if self.is_authenticated:
            self.node.get_logger().info(f'alarm_write:{value[0]}')
            self.node.config[1] = bool(value[0])
            self.node.publish_config()
    
    def volume_write_config(self, value,options):
        if self.is_authenticated:
            self.node.get_logger().info(f'volume_write:{value[0]}')
            self.node.config[2] = value[0]
            self.node.publish_config()


