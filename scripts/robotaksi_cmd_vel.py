#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge,CvBridgeError
import cv2
from smart_can_msgs.msg import Rcunittoomux, Rcthrtdata, Autonomousbrakepedalcontrol, Autonomoushbmotorcontrol, Autonomoussteeringmotcontrol
import time
from std_msgs.msg import Float64
import numpy as np




class RobotaksiCmdVel(Node):
    def __init__(self):
        super().__init__("robotaksi_cmd_vel")
        
        self.subscriber_ = self.create_subscription(Twist, "/cmd_vel", self.callback_cmd_vel, 
                                                    10
                                                    )
        
        self.steering_pub = self.create_publisher(Autonomoussteeringmotcontrol, '/beemobs/AUTONOMOUS_SteeringMot_Control', 10)
        self.unittomux_pub = self.create_publisher(Rcunittoomux, '/beemobs/rc_unittoOmux', 10)
        self.thrt_pub = self.create_publisher(Rcthrtdata, '/beemobs/RC_THRT_DATA', 10)
        self.brake_pub = self.create_publisher(Autonomousbrakepedalcontrol, '/beemobs/AUTONOMOUS_BrakePedalControl', 10)
        self.hb_pub = self.create_publisher(Autonomoushbmotorcontrol, '/beemobs/AUTONOMOUS_HB_MotorControl', 10)
        self.steering_target_pub = self.create_publisher(Float64,'/beemobs/steering_target_value',10)
        self.msg_unittomux = Rcunittoomux()
        self.msg = Autonomousbrakepedalcontrol()
        self.msg_ = Rcthrtdata()
        self.steering_target_msg=Float64()
        
        
        self.get_logger().info("Cmd has started.")

    def callback_cmd_vel(self, msg):

        steering_msg = Autonomoussteeringmotcontrol()
        steering_msg.autonomous_steeringmot_en = 1
        if int(msg.angular.z)<0:
            if 127-msg.angular.z>255:
                steering_msg.autonomous_steeringmot_pwm=255
                print("a1")
            else:
                steering_msg.autonomous_steeringmot_pwm = int(127-msg.angular.z)
                print("a2")
        elif int(msg.angular.z)>0:
            if 0+msg.angular.z>127:
                steering_msg.autonomous_steeringmot_pwm=127
            else:
                steering_msg.autonomous_steeringmot_pwm = int(0 +msg.angular.z)
            print('b')
                
                
        #self.steering_target_pub.publish(self.steering_target_msg)
        self.steering_pub.publish(steering_msg)
        
        self.msg_.rc_thrt_pedal_position = 70
        # self.msg_.rc_thrt_pedal_press = 0
        
        #self.thrt_pub.publish(self.msg_)
        
        


def main(args=None):
    rclpy.init(args=args)
    node = RobotaksiCmdVel()
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()