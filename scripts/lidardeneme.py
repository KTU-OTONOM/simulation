#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from math import cos, sin, pi, floor, tan
import numpy as np

from simple_pid import PID



class LidarVisualizeNode(Node):

    def __init__(self):
        super().__init__("lidar_visualize_node")
        self.subscriber_ = self.create_subscription(LaserScan, "/scan", self.callback_lidar, 10)
        self.subscriber_ = self.create_subscription(Imu, "/imu/data", self.callback_Imu, 10)
        self.publisher_ = self.create_publisher(Twist, "autocar/cmd_vel",10)       
        self.pid = PID(-0.20, 0, 0, setpoint=0)
        self.nerde=False
        self.flag=0
        self.get_logger().info("Lidar node has started.")
        self.on_last = False

    def callback_Imu(self, msg):
        
        self.yon = msg.orientation.z
        print(msg.orientation.z)
        

    def callback_lidar(self, msg):
        
        # print("eğim", self.yon )
       
        b = msg.angle_increment
        #print(b)

        msgControl=Twist()

        lidarRanges = np.array(msg.ranges)
        lidarRadian = np.arange(0,len(msg.ranges)).astype('float')
        lidarDegree = (2*pi -lidarRadian)*b*180/pi
        
        on = any(lidarRanges[269:449] <= 1.9)
        sol = any(lidarRanges[538:719] <= 2.5)
        sag = any(lidarRanges[0:179] <= 2.5)
        sagoncapraz = all(lidarRanges[300:359] >= 5)
        soloncapraz = all(lidarRanges[359:400] >= 5)     
        onall = all(lidarRanges[0:719] >= 3)
        
        # print("on  " ,on ,end="  ")
        # print("sol ",sol ,end="  ")
        # print("sağ",sag  ,end="  ")
        # print("nerde",self.nerde ,end="  ")
        
        for ind,degree in enumerate(lidarDegree):

            diff=abs(-degree)
            range=lidarRanges[np.where(lidarDegree==degree)] 
            radians = degree * pi / 180.0
        
        msgControl.linear.x=  0.8


        if self.on_last ==False and on==True:
            # print("a")
            self.baslangic=self.yon
            

        if on ==True :
            
            if sol and sag ==True:
                print("ARACI DURDUR !!!!")
                msgControl.linear.x=-8.0
                time.sleep(2)
                msgControl.linear.x=0.0
                self.flag=0
            
            elif self.nerde==False:
                if sol==False :
                    # print("SOL TARAFTAN DEVAM ET") 
                    self.flag=1
                    if sagoncapraz==True:
                        self.nerde=True

            elif self.nerde==True:
                if sag==False :
                    # print("SAĞ TARAFTAN DEVAM ET")
                    self.flag=2
                    if soloncapraz==True:            
                        self.nerde=False
             

        print(self.yon)

        if self.flag==1: 
            msgControl.angular.z=200.0
            if onall == True:
                msgControl.angular.z=0.0
                msgControl.linear.x=0.9
                # time.sleep(1.2)
                if  on == False :
                    msgControl.linear.x=0.9


                    if not abs(self.baslangic-self.yon)>=0.0174532925199433:
                        # print("a")
                        msgControl.linear.x=0.6           
                        msgControl.angular.z=-3.0

                        # self.flag=0 
                
                
                          
        elif self.flag==2:
            msgControl.angular.z=-100.0
            if onall == True:
                msgControl.angular.z=0.0
                self.flag=0
          
        
               
        self.publisher_.publish(msgControl)        
        self.on_last = on

def main(args=None):
    rclpy.init(args=args)
    node = LidarVisualizeNode()
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()
