#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Pose

from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity

class TrafficLightsService(Node): #MODIFY NAME
    def __init__(self):
        super().__init__("traffic_lights_service") #MODIFY NAME
        
        self.declare_parameter("index","")
        
        self.yellow_light="""
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="traffic_light_yellow">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://traffic_light_yellow/meshes/model.dae</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://traffic_light_yellow/meshes/model.dae</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </visual>

    </link>
  </model>
</sdf>
"""

        self.green_light="""
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="traffic_light_green">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://traffic_light_green/meshes/model.dae</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://traffic_light_green/meshes/model.dae</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </visual>

    </link>
  </model>
</sdf>
"""

        self.red_light="""
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="traffic_light_red">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://traffic_light_red/meshes/model.dae</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://traffic_light_red/meshes/model.dae</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </visual>

    </link>
  </model>
</sdf>
"""

        client_delete=self.create_client(DeleteEntity, "/delete_entity")
        client_spawn=self.create_client(SpawnEntity, '/spawn_entity')

        request_delete=DeleteEntity.Request()
        request_spawn=SpawnEntity.Request()
        
        index=self.get_parameter("index").value
        request_spawn.name="traffic_light_red"+index        
        
        self.get_logger().info("Traffic Lights client has finished")
        
        #time.sleep(30)
        
        while True:
            request_delete.name=request_spawn.name
            request_spawn.name="traffic_light_yellow"+index
            request_spawn.reference_frame=request_delete.name
            request_spawn.xml=self.yellow_light
            client_spawn.call_async(request_spawn)
            client_delete.call_async(request_delete)
            
            time.sleep(5)
            
            request_delete.name=request_spawn.name
            request_spawn.name="traffic_light_green"+index
            request_spawn.reference_frame=request_delete.name
            request_spawn.xml=self.green_light
            client_spawn.call_async(request_spawn)
            client_delete.call_async(request_delete)
            
            time.sleep(20)
            
            request_delete.name=request_spawn.name
            request_spawn.name="traffic_light_yellow"+index
            request_spawn.reference_frame=request_delete.name
            request_spawn.xml=self.yellow_light
            client_spawn.call_async(request_spawn)
            client_delete.call_async(request_delete)
            
            time.sleep(5)
            
            request_delete.name=request_spawn.name
            request_spawn.name="traffic_light_red"+index
            request_spawn.reference_frame=request_delete.name
            request_spawn.xml=self.red_light
            client_spawn.call_async(request_spawn)
            client_delete.call_async(request_delete)
            
            time.sleep(10)
        
        
        
        
                      
    # def call_delete_model_service(self,name,last):
    #     client_delete = self.create_client(DeleteEntity, "/delete_entity")
    #     client_spawn=self.create_client(SpawnEntity,"/spawn_entity")
    #     while not client_delete.wait_for_service(1.0):
    #         self.get_logger().warn("Waiting for Server Add Two Ints...")

    #     request_delete = DeleteEntity.Request()
    #     request_spawn=SpawnEntity.Request()
    #     request_spawn.name=name+self.model_name
    #     request_delete.name=name+self.model_name
    #     request_spawn.reference_frame=self.model_name
    #     self.model_pose=Pose()
    #     self.model_pose.position.z=3.0
    #     request_spawn.initial_pose=self.model_pose
        
    #     if name =="green_light":
    #         if last==None:
    #             request_spawn.xml=self.green_light
    #             client_spawn.call_async(request_spawn)
    #             time.sleep(6)
    #             request_spawn.name="yellow_light"+self.model_name
    #             request_spawn.xml=self.yellow_light
    #             client_spawn.call_async(request_spawn)
    #             client_delete.call_async(request_delete)
    #         elif last=="yellow_light":
    #             request_spawn.xml=self.green_light
    #             client_spawn.call_async(request_spawn)
    #             request_delete.name="yellow_light"+self.model_name
    #             client_delete.call_async(request_delete)
    #             time.sleep(6)
    #             request_spawn.name="yellow_light"+self.model_name
    #             request_spawn.xml=self.yellow_light
    #             client_spawn.call_async(request_spawn)
    #             request_delete.name="green_light"+self.model_name
    #             client_delete.call_async(request_delete)
                
                
    #     elif name=="yellow_light":
    #         if last=="red_light":
    #             request_spawn.xml=self.yellow_light
    #             client_spawn.call_async(request_spawn)
    #             request_delete.name="red_light"+self.model_name
    #             client_delete.call_async(request_delete)
    #             time.sleep(4)
    #             request_spawn.name="green_light"+self.model_name
    #             request_spawn.xml=self.green_light
    #             client_spawn.call_async(request_spawn)
    #             request_delete.name="yellow_light"+self.model_name
    #             client_delete.call_async(request_delete)
    #         elif last=="green_light":
    #             request_spawn.xml=self.yellow_light
    #             client_spawn.call_async(request_spawn)
    #             request_delete.name="green_light"+self.model_name
    #             client_delete.call_async(request_delete)
    #             time.sleep(4)
    #             request_spawn.name="red_light"+self.model_name
    #             request_spawn.xml=self.red_light
    #             client_spawn.call_async(request_spawn)
    #             request_delete.name="yellow_light"+self.model_name
    #             client_delete.call_async(request_delete)
                
    #     elif name=="red_light":
    #         request_spawn.xml=self.red_light
    #         client_spawn.call_async(request_spawn)
    #         request_delete.name="yellow_light"+self.model_name
    #         client_delete.call_async(request_delete)
    #         time.sleep(8)
    #         request_spawn.name="yellow_light"+self.model_name
    #         request_spawn.xml=self.yellow_light
    #         client_spawn.call_async(request_spawn)
    #         request_delete.name="red_light"+self.model_name
    #         client_delete.call_async(request_delete)
            
def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightsService() #MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()