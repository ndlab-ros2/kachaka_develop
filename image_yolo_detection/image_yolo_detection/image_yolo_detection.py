#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
import json


class ImageYoloDetection(Node):
    def __init__(self):
        super().__init__('image_yolo_detection')
        
        # Set QoS profile for image data transmission
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
        
        # Create publisher for detection results
        self.publisher = self.create_publisher(
            Image,
            '/image_yolo_detection/image',
            qos_profile
        )

        # Publish detected object information
        self.object_publisher = self.create_publisher(
            String,
            '/image_yolo_detection/objects',
            qos_profile
        )

        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = YOLO('yolov8s.pt')
        
        # Record previously detected objects
        self.last_detected_objects = set()
        
        self.get_logger().info("YOLOv8 node has started.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error("Error converting image: " + str(e))
            return

        # Perform object detection using YOLOv8
        results = self.model(cv_image)

        # Draw detection results on the image
        annotated_image = results[0].plot()

        # Get detected objects
        detected_objects = set()
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = result.names[class_id]
                detected_objects.add(class_name)

        # Publish information if new objects are detected
        new_objects = detected_objects - self.last_detected_objects
        if new_objects:
            # Publish detected object information in JSON format
            object_info = {
                "objects": list(new_objects),
                "timestamp": self.get_clock().now().nanoseconds * 1e-9
            }
            object_msg = String()
            object_msg.data = json.dumps(object_info)
            self.object_publisher.publish(object_msg)
            
            self.get_logger().info(f"Detected objects: {new_objects}")

        # Update detection results
        self.last_detected_objects = detected_objects

        # Convert detection results to ROS Image message and publish
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.publisher.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error("Error converting detection results: " + str(e))

        # Display detection results in a window
        cv2.imshow("YOLOv8 Detection", annotated_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageYoloDetection()
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