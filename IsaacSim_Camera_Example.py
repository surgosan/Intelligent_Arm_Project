#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        # Subscribe to the camera topic. May change based on camera
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',  # This is the topic. Make sure it matches the camera
            self.listener_callback,
            10)
        self.subscription
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Received image frame')
        try:
            # ROS Image to OpenCV image (BGR image format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Make grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # edge detection
        edges = cv2.Canny(gray_image, 100, 200)

        # Display
        cv2.imshow("Camera Image", cv_image)
        cv2.imshow("Edge Detection", edges)
        cv2.waitKey(1)  # Refresh

def main(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()
    try:
        rclpy.spin(camera_processor)
    except KeyboardInterrupt:
        pass
    finally:
        camera_processor.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
