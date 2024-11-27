#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge,CvBridgeError
import cv2
from teknofest_pkg.lane_model import LaneModel
from teknofest_pkg.image_preparator import ImagePreparator
from teknofest_pkg.inverse_perspective_mapping import InversePerspectiveMapping




class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        
        self.qos_profile=QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscriber_ = self.create_subscription(CompressedImage, "/zed_cam/camera_sensor/image_raw/compressed", self.callback_camera, 
                                                    10                                                    
                                                    )
        self.img_prep = ImagePreparator()
        self.ipm = InversePerspectiveMapping()
        self.bridge = CvBridge()
        
        self.above_value=0.58
        self.below_value=0.1
        self.side_value=0.3
        
        self.deviation=5
        self.border=0
        
        self.threshold_low=50
        self.threshold_high=150
        self.aperture=3
        
        self.lane_model=LaneModel(50,1,10)
        
                
        self.get_logger().info("Camera has started.")

    def callback_camera(self, msg):
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        except CvBridgeError as e:
            print(e)
            
        cv_image=cv2.resize(cv_image,(640,480),interpolation=cv2.INTER_CUBIC)
        
        # Inverse Perspective Mapping
        self.ipm.initializeTransformationMatrix(cv_image)
        warped = self.ipm.warp(cv_image)

        # crop
        cropped = self.img_prep.crop(warped, self.above_value, self.below_value, self.side_value)

        # grayscale
        gray = self.img_prep.grayscale(cropped)

        # blur
        blurred = self.img_prep.blur(gray, (self.deviation, self.deviation), self.border)

        # canny
        canny = self.img_prep.edge_detection(blurred, self.threshold_low, self.threshold_high, self.aperture)

        heigth, width = canny.shape
        if width == 63:  # TODO dirty hack
            cv2.line(canny, (0, 4/2), (18/2, heigth), (0, 0, 0), 2)
            cv2.line(canny, (width, 4/2), (width - 18/2, heigth), (0, 0, 0), 2)
        else:
            cv2.line(canny, (0, 4), (18, heigth), (0, 0, 0), 2)
            cv2.line(canny, (width, 4), (width - 18, heigth), (0, 0, 0), 2)

        # Lane Detection
        canny = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
        self.lane_model.update_segments(canny.copy())
        self.lane_model.draw_segments(canny)
        state_point_x = self.lane_model.state_point_x()
            
        cv2.imshow("img",cv_image)
        cv2.imshow("canny",canny)
        
        
        key=cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()