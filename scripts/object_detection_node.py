#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")
        
        self.model = cv2.dnn.readNetFromDarknet("/home/alper/yolov7.cfg","/home/alper/yolov7_best.weights")

        self.layers= self.model.getLayerNames()
        
        self.output_layer = [self.layers[layer - 1] for layer in self.model.getUnconnectedOutLayers()]
        
        self.labels=["Park Etmek Yasaktir", "Girisi Olmayan Yol", "Sola Donulmaz", "30", "Dur", "20", "Yaya Geicid", "Ileri Sag", "Yesil Isik",
              "Ileri Sol", "Kirmizi Isik", "Sari Isik", "Saga Donulmez", "Mecburi Sag", "Gidis Donus", "Durak",
              "Donel Kavsak", "Park Yeri", "Ileri", "Mecburi Sol", "Engelli Park"]

        self.subscriber_ = self.create_subscription(Image,
                                                    "/zed_cam/camera_sensor/image_raw",
                                                    self.callback_camera_n_control,
                                                    10)  
        
        self.image_width=672
        self.image_height=376
        
        self.image_info = np.array([self.image_width, self.image_height, self.image_width, self.image_height])

        self.bridge=CvBridge()

        self.get_logger().info("Camera object detection has started.")
    def callback_camera_n_control(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        except CvBridgeError as e:
            print(e)
            
        input_image = cv2.dnn.blobFromImage(cv_image, 1 / 255, (416, 416), swapRB=True, crop=False)

            
        self.model.setInput(input_image)
        detection_layers = self.model.forward(self.output_layer)

        indexes = []
        boxes = []
        confidences = []

        for detection_layer in detection_layers:
            for object_detection in detection_layer:

                scores = object_detection[5:]
                predicted_id = np.argmax(scores)
                confidence = scores[predicted_id]

                if (confidence > 0.6):
                    label = self.labels[predicted_id]
                    bounding_box = object_detection[0:4] * self.image_info
                    (box_centerX, box_centerY, box_width, box_height) = bounding_box.astype("int")

                    start_x = int(box_centerX - (box_width / 2))
                    start_y = int(box_centerY - (box_height / 2))

                    end_x = (start_x + box_width)
                    end_y = (start_y + box_height)

                    indexes.append(predicted_id)
                    boxes.append([start_x, start_y, int(box_width), int(box_height)])
                    confidences.append(float(confidence))

        max_ids = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        for max_id in max_ids:
            max_class_id = max_id
            box = boxes[max_class_id]

            predicted_id = indexes[max_class_id]
            confidence = confidences[max_class_id]
            label = self.labels[predicted_id]

            cv2.rectangle(cv_image, (start_x, start_y), (end_x, end_y), (0, 255, 0), 1)
            cv2.putText(cv_image, label, (start_x, start_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)


        cv2.imshow("video", cv_image)
        key=cv2.waitKey(1)
            


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()