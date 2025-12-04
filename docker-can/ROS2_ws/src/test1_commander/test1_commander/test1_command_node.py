import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
import threading

class Test1CommandNode(Node):
    """
    This node publishes a predefined sequence of commands for lights, turn signals,
    and the horn to test the vehicle's auxiliary functions.
    """
    def __init__(self):
        super().__init__('test1_command_node')
        self.get_logger().info('Test 1 Command Node has started.')
        
        # --- Publishers ---
        self.cmd_topic_prefix = '/cmd'
        self.lights_pub = self.create_publisher(Bool, f'{self.cmd_topic_prefix}/ref_lights', 10)
        self.turn_l_pub = self.create_publisher(Bool, f'{self.cmd_topic_prefix}/ref_turning_light_l', 10)
        self.turn_r_pub = self.create_publisher(Bool, f'{self.cmd_topic_prefix}/ref_turning_light_r', 10)
        self.horn_pub = self.create_publisher(Bool, f'{self.cmd_topic_prefix}/ref_horn', 10)

        # To avoid blocking the ROS2 spin, we run the sequence in a separate thread.
        self.command_thread = threading.Thread(target=self.run_command_sequence)
        self.command_thread.start()

    def run_command_sequence(self):
        """
        Runs the full sequence of commands with delays.
        """
        self.get_logger().info("Starting command sequence in 2 seconds...")
        time.sleep(2) # Give a moment for publishers to establish connections

        msg = Bool()

        # --- Sequence Step 1: Lights ---
        self.get_logger().info("Turning lights ON.")
        msg.data = True
        self.lights_pub.publish(msg)
        time.sleep(3)

        self.get_logger().info("Turning lights OFF.")
        msg.data = False
        self.lights_pub.publish(msg)
        time.sleep(1)

        # --- Sequence Step 2: Turning Lights ---
        self.get_logger().info("Turning HAZARD lights ON.")
        msg.data = True
        self.turn_l_pub.publish(msg)
        self.turn_r_pub.publish(msg)
        time.sleep(3)
        
        self.get_logger().info("Turning HAZARD lights OFF.")
        msg.data = False
        self.turn_l_pub.publish(msg)
        self.turn_r_pub.publish(msg)
        time.sleep(3)

        # --- Sequence Step 3: Horn ---
        self.get_logger().info("Activating HORN.")
        msg.data = True
        self.horn_pub.publish(msg)
        time.sleep(1)

        self.get_logger().info("Deactivating HORN.")
        msg.data = False
        self.horn_pub.publish(msg)

        self.get_logger().info("Command sequence finished. Node will remain active. Press Ctrl-C to exit.")

def main(args=None):
    rclpy.init(args=args)
    test_node = Test1CommandNode()
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
