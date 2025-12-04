import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, UInt8
import time
import threading

class Test2CommandNode(Node):
    """
    This node publishes a predefined sequence of driving and auxiliary commands
    to test vehicle control.
    """
    def __init__(self):
        super().__init__('test2_command_node')
        self.get_logger().info('Test 2 Driving Command Node has started.')
        
        # --- Publishers ---
        self.cmd_topic_prefix = '/cmd'
        
        # Driving commands
        self.steer_angle_pub = self.create_publisher(Float64, f'{self.cmd_topic_prefix}/ref_steering_wheel_angle', 10)
        self.throttle_pub = self.create_publisher(Float64, f'{self.cmd_topic_prefix}/ref_throttle', 10)
        self.brake_pub = self.create_publisher(Float64, f'{self.cmd_topic_prefix}/ref_brake_pressure', 10)
        self.speed_pub = self.create_publisher(Float64, f'{self.cmd_topic_prefix}/ref_car_speed', 10)
        
        # Enable flags
        self.steer_enable_pub = self.create_publisher(Bool, f'{self.cmd_topic_prefix}/ref_steering_wheel_enable', 10)
        self.throttle_enable_pub = self.create_publisher(Bool, f'{self.cmd_topic_prefix}/ref_throttle_enable', 10)
        self.brakes_enable_pub = self.create_publisher(Bool, f'{self.cmd_topic_prefix}/ref_brakes_enable', 10)
        self.speed_enable_pub = self.create_publisher(Bool, f'{self.cmd_topic_prefix}/ref_speed_control_enable', 10)
        
        # Auxiliary commands
        self.turn_l_pub = self.create_publisher(Bool, f'{self.cmd_topic_prefix}/ref_turning_light_l', 10)
        self.turn_r_pub = self.create_publisher(Bool, f'{self.cmd_topic_prefix}/ref_turning_light_r', 10)
        self.gear_pub = self.create_publisher(UInt8, f'{self.cmd_topic_prefix}/ref_drive_neutral_reverse', 10)
        
        # Run the sequence in a separate thread to avoid blocking ROS2 operations
        self.command_thread = threading.Thread(target=self.run_command_sequence)
        self.command_thread.start()

    def run_command_sequence(self):
        """Runs the full driving test sequence with specified delays."""
        self.get_logger().info("Starting driving sequence in 2 seconds...")
        time.sleep(2) # Wait for publishers to connect

        # Re-usable message objects
        bool_msg = Bool()
        float_msg = Float64()
        uint8_msg = UInt8()

        # --- Sequence Start ---
        self.get_logger().info("1. Turning HAZARD lights ON.")
        bool_msg.data = True
        self.turn_l_pub.publish(bool_msg)
        self.turn_r_pub.publish(bool_msg)
        time.sleep(5)

        self.get_logger().info("2. Setting gear to FORWARD (value: 2).")
        uint8_msg.data = 2
        self.gear_pub.publish(uint8_msg)
        time.sleep(1)

        self.get_logger().info("3. ENABLING all 4 driving commands in order (brakes, steering, throttle, and speed control).")
        bool_msg.data = True
        self.brakes_enable_pub.publish(bool_msg)
        time.sleep(0.2)
        self.steer_enable_pub.publish(bool_msg)
        time.sleep(0.2)
        self.throttle_enable_pub.publish(bool_msg)
        time.sleep(0.2)
        self.speed_enable_pub.publish(bool_msg)
        time.sleep(1)
        time.sleep(1)

        self.get_logger().info("4. Setting velocity to gradually accelerate to 8km/h and then gradually decelerate to 0km/h.")
        speed_profile = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0]
        for speed in speed_profile:
            float_msg.data = speed
            self.speed_pub.publish(float_msg)
            self.get_logger().info(f"  Setting speed to: {speed} km/h") # Optional: better logging
            time.sleep(0.5)
        self.get_logger().info("5. Speed profile complete. Vehicle stopped.")

        
        self.get_logger().info("6. Setting speed to 0 km/h and applying brakes (5 bar).")
        float_msg.data = 0.0
        self.speed_pub.publish(float_msg)
        float_msg.data = 5.0
        self.brake_pub.publish(float_msg)
        time.sleep(3)
        
        self.get_logger().info("7. Releasing brakes.")
        float_msg.data = 0.0
        self.brake_pub.publish(float_msg)
        time.sleep(8)
        
        self.get_logger().info("8. Increase gradually throttle to 20% and then gradually setting in back to 0%.")
        throttle_profile = [2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 18.0, 16.0, 14.0, 12.0, 10.0, 8.0, 6.0, 4.0, 2.0, 0.0]
        for throttle in speed_profile:
            float_msg.data = throttle
            self.throttle_pub.publish(float_msg)
            self.get_logger().info(f"  Setting throttle to: {throttle} %")
            time.sleep(0.5)
        self.get_logger().info("9. Throttle profile complete. Vehicle stopped.")

        self.get_logger().info("10. Setting throttle to 0% and applying brakes (5 bar).")
        float_msg.data = 0.0
        self.throttle_pub.publish(float_msg)
        float_msg.data = 5.0
        self.brake_pub.publish(float_msg)
        time.sleep(3)
        
        self.get_logger().info("11. Releasing brakes.")
        float_msg.data = 0.0
        self.brake_pub.publish(float_msg)
        time.sleep(8)
        
        self.get_logger().info("12. Increase steering angle gradually to 90 degrees.")
        angle_profile = [2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5, 25.0, 27.5, 30.0, 27.5, 25.0, 22.5, 20.0, 17.5, 15.0, 12.5, 10.0, 7.5, 5.0, 2.5]
        for angle in angle_profile:
            float_msg.data = angle
            self.steer_angle_pub.publish(float_msg)
            self.get_logger().info(f"  Setting steering angle to: {angle} %")
            time.sleep(0.25)
        self.get_logger().info("13. Steering angle profile complete.")
        
        self.get_logger().info("14. DISABLING all driving commands.")
        bool_msg.data = False
        self.speed_enable_pub.publish(bool_msg)
        time.sleep(0.2)
        self.throttle_enable_pub.publish(bool_msg)
        time.sleep(0.2)
        self.steer_enable_pub.publish(bool_msg)
        time.sleep(0.2)
        self.brakes_enable_pub.publish(bool_msg)
        time.sleep(1)
        
        self.get_logger().info("15. Setting gear to NEUTRAL (value: 0).")
        uint8_msg.data = 0
        self.gear_pub.publish(uint8_msg)
        time.sleep(1)
        
        self.get_logger().info("16. Turning HAZARD lights OFF.")
        bool_msg.data = False
        self.turn_l_pub.publish(bool_msg)
        self.turn_r_pub.publish(bool_msg)
        
        self.get_logger().info("Command sequence finished. Node will remain active. Press Ctrl-C to exit.")


def main(args=None):
    rclpy.init(args=args)
    test_node = Test2CommandNode()
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
