from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_mqtt_bridge',
            executable='car',
            name='ros2_to_mqtt_bridge',
            parameters=[
                {'ros_topic': '/antenna_rear/gpsfix'},
                {'mqtt_topic': 'gps'},
                {'mqtt_host': '89c06e07cd3b405794aa072e33b5f180.s1.eu.hivemq.cloud'},
                {'mqtt_port': 8883},
                {'mqtt_username': 'spoke9'},
                {'mqtt_password': 'I473lEpo3m4L'},
                {'mqtt_qos': 1},
                {'mqtt_retain': False},
                {'client_id': 'ros2-mqtt-bridge'},
                {'keepalive': 60},
                {'log_every_publish': True},
            ]
        )
    ])
