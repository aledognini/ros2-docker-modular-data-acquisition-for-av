#!/usr/bin/env python3
import json
import ssl
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from gps_msgs.msg import GPSFix 
import paho.mqtt.client as mqtt


class GpsToMqttBridge(Node):
    def __init__(self):
        super().__init__("gps_to_mqtt_bridge")

        # ---- Parameters (override via launch/CLI if needed)
        self.declare_parameter("ros_topic", "/antenna_rear/gpsfix")
        self.declare_parameter("mqtt_topic", "")  # if empty, derived from ros_topic
        self.declare_parameter("mqtt_host", "89c06e07cd3b405794aa072e33b5f180.s1.eu.hivemq.cloud")
        self.declare_parameter("mqtt_port", 8883)
        self.declare_parameter("mqtt_username", "spoke9")
        self.declare_parameter("mqtt_password", "I473lEpo3m4L")
        self.declare_parameter("mqtt_qos", 1)
        self.declare_parameter("mqtt_retain", False)
        self.declare_parameter("client_id", "ros2-mqtt-gps-bridge")
        self.declare_parameter("keepalive", 60)
        self.declare_parameter("log_every_publish", True)
        self.declare_parameter("only_valid_fix", False)  # set True to skip invalid fixes

        ros_topic = self.get_parameter("ros_topic").get_parameter_value().string_value
        mqtt_topic_param = self.get_parameter("mqtt_topic").get_parameter_value().string_value
        self.mqtt_topic = mqtt_topic_param if mqtt_topic_param else ros_topic  # keep same name by default
        self.mqtt_host = self.get_parameter("mqtt_host").get_parameter_value().string_value
        self.mqtt_port = self.get_parameter("mqtt_port").get_parameter_value().integer_value
        self.mqtt_username = self.get_parameter("mqtt_username").get_parameter_value().string_value
        self.mqtt_password = self.get_parameter("mqtt_password").get_parameter_value().string_value
        self.mqtt_qos = self.get_parameter("mqtt_qos").get_parameter_value().integer_value
        self.mqtt_retain = self.get_parameter("mqtt_retain").get_parameter_value().bool_value
        self.client_id = self.get_parameter("client_id").get_parameter_value().string_value
        self.keepalive = self.get_parameter("keepalive").get_parameter_value().integer_value
        self.log_every_publish = self.get_parameter("log_every_publish").get_parameter_value().bool_value
        self.only_valid_fix = self.get_parameter("only_valid_fix").get_parameter_value().bool_value

        # ---- MQTT client (TLS)
        self._mqtt = mqtt.Client(client_id=self.client_id, clean_session=True)
        self._mqtt.username_pw_set(self.mqtt_username, self.mqtt_password)
        self._mqtt.tls_set(cert_reqs=ssl.CERT_REQUIRED)
        self._mqtt.tls_insecure_set(False)
        self._mqtt.will_set(f"{self.mqtt_topic}/bridge_status", payload="offline", qos=1, retain=True)
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_disconnect = self._on_disconnect

        try:
            self._mqtt.connect(self.mqtt_host, self.mqtt_port, keepalive=self.keepalive)
        except Exception as e:
            self.get_logger().error(f"MQTT initial connect failed: {e}")

        self._mqtt_loop_thread = threading.Thread(target=self._mqtt.loop_forever, kwargs={"retry_first_connection": True}, daemon=True)
        self._mqtt_loop_thread.start()

        # Announce
        self._publish_status("online")

        # ---- ROS sub
        self._sub = self.create_subscription(GPSFix, ros_topic, self._gps_cb, 10)
        self.get_logger().info(f"Bridging ROS '{ros_topic}' (NavSatFix) -> MQTT '{self.mqtt_topic}' as JSON tuple [lat, lon]")

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT connected.")
            self._publish_status("online")
        else:
            self.get_logger().error(f"MQTT connect error rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warn(f"MQTT unexpected disconnect rc={rc}")

    def _publish_status(self, status: str):
        try:
            self._mqtt.publish(f"{self.mqtt_topic}/bridge_status", status, qos=1, retain=True)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish bridge status '{status}': {e}")

    def _gps_cb(self, msg: NavSatFix):
        # Optionally skip invalid fixes
        if self.only_valid_fix and msg.status.status < 0:
            return
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        speed = float (msg.speed)
        timestamp_sec = int (msg.header.stamp.sec)
        timestamp_nsec = int (msg.header.stamp.nanosec)
        payload = json.dumps([lat, lon, speed, timestamp_sec, timestamp_nsec])  # 2-element array: tuple-like

        try:
            res = self._mqtt.publish(self.mqtt_topic, payload=payload, qos=self.mqtt_qos, retain=self.mqtt_retain)
            if res.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().warn(f"MQTT publish returned rc={res.rc}")
            elif self.log_every_publish:
                self.get_logger().info(f"Published {self.mqtt_topic}: {payload}")
        except Exception as e:
            self.get_logger().error(f"MQTT publish failed: {e}")

    def destroy_node(self):
        self._publish_status("offline")
        try:
            self._mqtt.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GpsToMqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
