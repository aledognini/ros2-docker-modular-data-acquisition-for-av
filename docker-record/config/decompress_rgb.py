#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageDecompressor(Node):
    def __init__(self):
        super().__init__('image_decompressor_node')
        
        # 1. Crea il Publisher per il topic RAW
        self.publisher_ = self.create_publisher(
            Image, 
            '/zed/zed_node/rgb/image_rect_color', 
            10)
        
        # 2. Crea il Subscriber per il topic COMPRESSED
        self.subscription = self.create_subscription(
            CompressedImage,
            '/zed/zed_node/rgb/image_rect_color/compressed',
            self.decompress_callback,
            10)
            
        self.bridge = CvBridge()
        self.get_logger().info('Decompressore RGB avviato. In ascolto su .../compressed')

    def decompress_callback(self, msg):
        try:
            # 3. Decomprimi il messaggio usando OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # 4. Riconverti l'immagine OpenCV in un messaggio ROS Image
            # Manteniamo lo stesso timestamp e frame_id del messaggio originale
            raw_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            raw_msg.header = msg.header
            
            # 5. Pubblica il messaggio RAW
            self.publisher_.publish(raw_msg)
            
        except Exception as e:
            self.get_logger().error(f'Errore durante la decompressione: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageDecompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
