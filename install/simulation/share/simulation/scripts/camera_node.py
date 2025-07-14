#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscriber_ = self.create_subscription(
            CompressedImage,
            "/zed_cam/camera_sensor/image_raw/compressed",
            self.callback_camera,
            qos_profile=self.qos_profile
        )

        self.bridge = CvBridge()
        self.get_logger().info("Camera subscription has started.")

    def callback_camera(self, msg):
        try:
            # ROS mesajını OpenCV BGR görüntüye dönüştür
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
            return

        # Orijinal frame’i ekranda göster
        cv2.imshow("Camera Frame", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
