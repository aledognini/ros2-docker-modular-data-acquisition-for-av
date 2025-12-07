from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_mqtt_bridge',
            executable='car2',
            name='ros2_to_mqtt_bridge',
            parameters=[
                {'ros_topic': '/antenna_rear/gpsfix'},
                {'mqtt_topic': '/antenna_rear/gpsfix'},
                #{'mqtt_topic': 'gps'},
                {'mqtt_host': '10.45.0.1'},
                {'mqtt_port': 1883},
                {'mqtt_qos': 0},
                {'mqtt_retain': False},
                {'client_id': 'ros2-mqtt-bridge'},
                {'keepalive': 60},
                {'log_every_publish': True},
            ]
        )
    ])
