#!/usr/bin/env python3
import os
import ssl
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import paho.mqtt.client as mqtt


class Ros2ToMqttBridge(Node):
    def __init__(self):
        super().__init__("ros2_to_mqtt_bridge")

        # --- ROS params (can be overridden via CLI/launch) ---
        self.declare_parameter("ros_topic", "collision")
        self.declare_parameter("mqtt_topic", "collision")
        self.declare_parameter("mqtt_host", "89c06e07cd3b405794aa072e33b5f180.s1.eu.hivemq.cloud")
        self.declare_parameter("mqtt_port", 8883)
        self.declare_parameter("mqtt_username", "spoke9")
        self.declare_parameter("mqtt_password", "I473lEpo3m4L")
        self.declare_parameter("mqtt_qos", 1)
        self.declare_parameter("mqtt_retain", False)
        self.declare_parameter("client_id", "ros2-mqtt-bridge")
        self.declare_parameter("keepalive", 60)
        self.declare_parameter("log_every_publish", True)

        self.ros_topic = self.get_parameter("ros_topic").get_parameter_value().string_value
        self.mqtt_topic = self.get_parameter("mqtt_topic").get_parameter_value().string_value
        self.mqtt_host = self.get_parameter("mqtt_host").get_parameter_value().string_value
        self.mqtt_port = self.get_parameter("mqtt_port").get_parameter_value().integer_value
        self.mqtt_username = self.get_parameter("mqtt_username").get_parameter_value().string_value
        self.mqtt_password = self.get_parameter("mqtt_password").get_parameter_value().string_value
        self.mqtt_qos = self.get_parameter("mqtt_qos").get_parameter_value().integer_value
        self.mqtt_retain = self.get_parameter("mqtt_retain").get_parameter_value().bool_value
        self.client_id = self.get_parameter("client_id").get_parameter_value().string_value
        self.keepalive = self.get_parameter("keepalive").get_parameter_value().integer_value
        self.log_every_publish = self.get_parameter("log_every_publish").get_parameter_value().bool_value

        # --- MQTT client setup (TLS) ---
        self._mqtt = mqtt.Client(client_id=self.client_id, clean_session=True)
        self._mqtt.username_pw_set(self.mqtt_username, self.mqtt_password)
        # Use system CA store; require valid cert
        self._mqtt.tls_set(cert_reqs=ssl.CERT_REQUIRED)
        self._mqtt.tls_insecure_set(False)
        # Last Will (optional): announce bridge offline
        self._mqtt.will_set(f"{self.mqtt_topic}/bridge_status", payload="offline", qos=1, retain=True)

        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_disconnect = self._on_disconnect

        # Run MQTT network loop in a background thread
        self._mqtt_loop_thread = threading.Thread(target=self._mqtt_loop, daemon=True)
        try:
            self._mqtt.connect(self.mqtt_host, self.mqtt_port, keepalive=self.keepalive)
        except Exception as e:
            self.get_logger().error(f"MQTT initial connect failed: {e}")

        self._mqtt_loop_thread.start()

        # Publish that we’re online
        self._publish_bridge_status("online")

        # --- ROS subscriber ---
        self.subscription = self.create_subscription(Bool, self.ros_topic, self._ros_callback, 10)
        self.get_logger().info(
            f"Bridging ROS '{self.ros_topic}' (Bool) -> MQTT '{self.mqtt_topic}' "
            f"(payload 'true'/'false') via {self.mqtt_host}:{self.mqtt_port}"
        )

    # MQTT event handlers
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT connected.")
            self._publish_bridge_status("online")
        else:
            self.get_logger().error(f"MQTT connect error rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warn(f"MQTT unexpected disconnect rc={rc}. Reconnecting...")

    def _mqtt_loop(self):
        # Loop forever; paho handles reconnects with loop_forever/loop_start
        try:
            self._mqtt.loop_forever(retry_first_connection=True)
        except Exception as e:
            self.get_logger().error(f"MQTT loop error: {e}")

    def _publish_bridge_status(self, status: str):
        try:
            self._mqtt.publish(f"{self.mqtt_topic}/bridge_status", status, qos=1, retain=True)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish bridge status '{status}': {e}")

    # ROS -> MQTT
    def _ros_callback(self, msg: Bool):
        payload = "true" if msg.data else "false"
        try:
            result = self._mqtt.publish(self.mqtt_topic, payload=payload, qos=self.mqtt_qos, retain=self.mqtt_retain)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().warn(f"MQTT publish returned rc={result.rc}")
            elif self.log_every_publish:
                self.get_logger().info(f"Published MQTT '{self.mqtt_topic}': {payload}")
        except Exception as e:
            self.get_logger().error(f"MQTT publish failed: {e}")

    def destroy_node(self):
        # Graceful shutdown
        self._publish_bridge_status("offline")
        try:
            self._mqtt.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Ros2ToMqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

