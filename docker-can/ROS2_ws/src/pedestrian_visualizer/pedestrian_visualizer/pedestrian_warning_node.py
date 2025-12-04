import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class PedestrianWarningNode(Node):
    """
    Visualizes a pedestrian warning signal in RViz2 as a 2D colored circle with a legend.
    - Green: No pedestrian detected (last message was false).
    - Red: Pedestrian detected (last message was true).
    - Yellow: No data received yet.
    """
    def __init__(self):
        super().__init__('pedestrian_warning_node')
        self.get_logger().info('Pedestrian Warning Visualizer has started.')

        # Stores the last received boolean value, None if nothing received yet
        self.last_warning_state = None

        # Subscriber to the warning topic
        self.subscription = self.create_subscription(
            Bool,
            '/pedestrian_warning',
            self.warning_callback,
            10)

        # Publisher for the visualization markers
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Timer to periodically publish the markers
        self.timer = self.create_timer(0.1, self.publish_markers) # 10 Hz

    def warning_callback(self, msg):
        """Callback function to update the warning state."""
        self.last_warning_state = msg.data

    def publish_markers(self):
        """Creates and publishes the 2D circle and legend markers."""
        marker_array = MarkerArray()
        
        # --- 2D Circle Marker ---
        circle_marker = Marker()
        circle_marker.header.frame_id = "base_link"
        circle_marker.header.stamp = self.get_clock().now().to_msg()
        circle_marker.ns = "pedestrian_warning"
        circle_marker.id = 0
        circle_marker.type = Marker.CYLINDER # Use a cylinder for a 2D circle
        circle_marker.action = Marker.ADD
        circle_marker.pose.position.x = 2.0 # Place it 2 meters in front
        circle_marker.pose.position.z = 0.0 # Place it flat on the ground
        circle_marker.pose.orientation.w = 1.0

        # Set the scale: x and y are the diameter, z is the height
        circle_marker.scale.x = 1.5  # 1.5 meter diameter
        circle_marker.scale.y = 1.5  # 1.5 meter diameter
        circle_marker.scale.z = 0.01 # Make it very thin (1 cm) to look 2D
        
        # Determine color based on state
        if self.last_warning_state is None:
            circle_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0) # Yellow
        elif self.last_warning_state:
            circle_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0) # Red
        else:
            circle_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0) # Green
        
        circle_marker.lifetime.sec = 1
        marker_array.markers.append(circle_marker)

        # --- Legend Markers ---
        legend_texts = [
            ("● RED=Pedestrian", ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)),
            ("● GREEN=Clear_Path", ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)),
            ("● YELLOW=No_Data", ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
        ]

        for i, (text, color) in enumerate(legend_texts):
            legend_marker = Marker()
            legend_marker.header.frame_id = "base_link"
            legend_marker.header.stamp = self.get_clock().now().to_msg()
            legend_marker.ns = "legend"
            legend_marker.id = i + 1 # Unique ID
            legend_marker.type = Marker.TEXT_VIEW_FACING
            legend_marker.action = Marker.ADD
            legend_marker.pose.position.x = 2.0
            legend_marker.pose.position.y = 1.5 - (i * 0.3) # Stack them vertically
            legend_marker.pose.position.z = 0.5
            legend_marker.scale.z = 0.2 # Text height
            legend_marker.color = color
            legend_marker.text = text
            legend_marker.lifetime.sec = 1
            marker_array.markers.append(legend_marker)

        # Publish the complete array
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PedestrianWarningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
