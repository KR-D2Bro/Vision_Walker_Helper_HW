from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_pkg',
            executable='imu_pub',
            name='imu_sensor'
        ),
        Node(
            package='sensor_pkg',
            executable='illuminance_pub',
            name='illuminance_sensor'
        ),
        Node(
            package='sensor_pkg',
            executable='battery_pub',
            name='battery_sensor'
        ),
        Node(
            package='actuator_pkg',
            executable='light_srv',
            name='light_actuator'
        ),
        Node(
            package='actuator_pkg',
            executable='buzzer_srv',
            name='buzzer_actuator'
        ),
        Node(
            package='app_interface_pkg',
            executable='ble_node',
            name='ble_node'
        ),
        Node(
            package='decision_pkg',
            executable='decision_node',
            name='decision_node'
        ),
        Node(
            package='vision_pkg',
            executable='vision_node',
            name='vision_node'
        ),

    ])

