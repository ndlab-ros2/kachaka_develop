#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class ImageGrayProcessor(Node):
    def __init__(self):
        super().__init__('image_gray_processor')

        # Set QoS profile
        qos_profile = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        ) 
               
        # Subscribe to camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/kachaka/front_camera/image_raw',
            self.image_callback,
            qos_profile
        )
        self.bridge = CvBridge()
        self.get_logger().info("ImageGrayProcessor has started.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Display grayscale image
            cv2.imshow("Grayscale Image", gray_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageGrayProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()