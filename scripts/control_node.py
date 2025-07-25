#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select

import tty, termios
  
msg = """
Control RasCar!
---------------------------
Moving around:
        w    e
   a    s    d
        x

s : force stop
e : stop rotational movement

CTRL-C to quit
"""



class ControlNode(Node): #MODIFY NAME
    def __init__(self):
        super().__init__("control_node") #MODIFY NAME
        
        
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel",10)
        msg=Twist()
              
        while True:

            self.settings = termios.tcgetattr(sys.stdin)
            key=self.getKey()

            if key=="w":
                msg.linear.x+=0.1
            elif key=="a":
                if msg.linear.x> 0.0:
                    msg.angular.z+=0.1
                else:
                    msg.angular.z-=0.1
            elif key=="s":
                msg.linear.x=0.0
                msg.angular.z=0.0
            elif key=="d":
                if msg.linear.x> 0.0:
                    msg.angular.z-=0.1
                else:
                    msg.angular.z+=0.1
            elif key=="x":
                msg.linear.x-=0.1
            elif key=='e':
                msg.angular.z=0.0
            else:
                if key=="\x03":
                    break

            self.publisher_.publish(msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        
    def getKey(self):

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


def main(args=None):
    print(msg)
    rclpy.init(args=args)
    node = ControlNode() #MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()