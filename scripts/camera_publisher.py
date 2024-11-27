#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage,Image
from cv_bridge import CvBridge,CvBridgeError
import cv2




class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.publisher_compressed = self.create_publisher(CompressedImage, "/camera/image/compressed",10)
        self.publisher_raw=self.create_publisher(Image,"/my_image_raw",10)
        self.timer_ = self.create_timer(0.01, self.callback_camera_publisher)
        
        self.bridge = CvBridge()
        
        self.cap=cv2.VideoCapture(0)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
                
        self.get_logger().info("Camera has started.")

    def callback_camera_publisher(self):
        
        _,cv_image=self.cap.read()
        
        cv_image=cv2.resize(cv_image,None,fx=1/4,fy=1/4)
        
        
        cv_image_bridge = self.bridge.cv2_to_compressed_imgmsg(cv_image,'jpg')
        cv_image_bridge_raw=self.bridge.cv2_to_imgmsg(cv_image,'bgr8')
        
        self.publisher_raw.publish(cv_image_bridge_raw)
        
        self.publisher_compressed.publish(cv_image_bridge)
            
        #cv2.imshow("img",cv_image)
        
        key=cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()