#! usr/bin/env python3
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy


class camera_publisher(Node):
    def __init__(self):
        super().__init__("camera_pub")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.camera = cv2.VideoCapture("/dev/video0")
        self.bridgeObject = CvBridge()
        self.pub_ = self.create_publisher(CompressedImage,"camera_topic",qos_profile)
        self.timer_=self.create_timer(0.001,self.timer_callback)
        self.count = 0

    def timer_callback(self):
        success, frame = self.camera.read()
        frame = cv2.resize(frame,(820,640), interpolation=cv2.INTER_CUBIC)
        if success == True:
            ROS2ImageMessage = self.bridgeObject.cv2_to_compressed_imgmsg(frame)
            self.pub_.publish(ROS2ImageMessage)
        
        self.get_logger().info("publising image number %d" % self.count)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = camera_publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()