#!/usr/bin/python
import time
import rclpy
from std_msgs.msg import Bool
import paho.mqtt.client as paho
from paho import mqtt
import numpy as np

# --- ROS2 Initialization ---
# Initialize the ROS2 client library.
rclpy.init()

# Create a ROS2 node.
node = rclpy.create_node('mqtt_warning_bridge_node')

# Create a publisher for the /pedestrian_warning topic.
publisher = node.create_publisher(Bool, '/pedestrian_warning', 10)

node.get_logger().info('ROS2 node and publisher for /pedestrian_warning created.')


# --- MQTT Callback Functions ---

# the callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        node.get_logger().info("Connected to MQTT Broker successfully!")
        warning_topic = "collision"
        client.subscribe(warning_topic, qos=1)
        node.get_logger().info(f"Subscribed to MQTT topic: '{warning_topic}'")
    else:
        node.get_logger().error(f"Failed to connect to MQTT, return code {rc}")

# the callback for when a message is received from the server
def on_message(client, userdata, msg):
    try:
        # 1. decode the message payload from bytes to a lowercase string.
        payload_str = msg.payload.decode('utf-8').lower()
        node.get_logger().info(f"Received message from MQTT topic '{msg.topic}': '{payload_str}'")

        # 2. create a ROS2 Bool message.
        ros_message = Bool()

        # 3. convert the received string to a boolean value.
        # this flexibly handles 'true', '1', or 'on' as True.
        ros_message.data = payload_str in ['true', '1', 'on']

        # 4. publish the message to the ROS2 topic.
        publisher.publish(ros_message)
        node.get_logger().info(f"Published to /pedestrian_warning: {ros_message.data}")

    except Exception as e:
        node.get_logger().error(f"Error processing MQTT message: {e}")


# --- Main Block ---

client = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
client.on_connect = on_connect
client.on_message = on_message

# enable TLS for a secure connection
client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
# set username and password
client.username_pw_set("spoke9", "I473lEpo3m4L")
# connect to HiveMQ Cloud on port 8883 (default for MQTT)
client.connect("89c06e07cd3b405794aa072e33b5f180.s1.eu.hivemq.cloud", 8883)

# loop_forever for simplicity
node.get_logger().info('Starting MQTT client loop... Press Ctrl+C to exit.')
client.loop_forever()

# for a clean shutdown+
node.get_logger().info('Shutting down ROS2 node.')
node.destroy_node()
rclpy.shutdown()
