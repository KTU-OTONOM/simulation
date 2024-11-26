#! usr/bin/env python3
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

class camera_subscriber(Node):
    def __init__(self):
        super().__init__("camera_sub")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.brideObject = CvBridge()
        self.sub_ = self.create_subscription(CompressedImage,"camera_topic",self.sub_callback,qos_profile)
        self.sub_
 
    def sub_callback(self,imageMessage):
        self.get_logger().info("the image frame is received")
        openCVImage = self.brideObject.compressed_imgmsg_to_cv2(imageMessage)
        cv2.imshow("Camera video",openCVImage)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = camera_subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()