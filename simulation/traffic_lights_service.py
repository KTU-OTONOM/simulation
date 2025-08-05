import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

class TrafficLightsService(Node):
    def __init__(self):
        super().__init__('traffic_lights_service')
        
        self.pose = Pose()
        self.pose.position.x = 22.0
        self.pose.position.y = 28.0
        self.pose.position.z = 0.1

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for spawn_entity service...")
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for delete_entity service...")

        self.run_loop()

    def spawn_model(self, name, sdf_xml):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf_xml
        req.initial_pose = self.pose
        req.reference_frame = "world"
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"{name} spawned")

    def delete_model(self, name):
        req = DeleteEntity.Request()
        req.name = name
        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"{name} deleted")

    def run_loop(self):
        red_xml = """<sdf version='1.7'><model name='traffic_light_red'>...model.dae...</model></sdf>"""
        yellow_xml = """<sdf version='1.7'><model name='traffic_light_yellow'>...model.dae...</model></sdf>"""
        green_xml = """<sdf version='1.7'><model name='traffic_light_green'>...model.dae...</model></sdf>"""

        while rclpy.ok():
            self.spawn_model("traffic_light_red", red_xml)
            time.sleep(10)
            self.delete_model("traffic_light_red")

            self.spawn_model("traffic_light_yellow", yellow_xml)
            time.sleep(5)
            self.delete_model("traffic_light_yellow")

            self.spawn_model("traffic_light_green", green_xml)
            time.sleep(20)
            self.delete_model("traffic_light_green")

            self.spawn_model("traffic_light_yellow", yellow_xml)
            time.sleep(5)
            self.delete_model("traffic_light_yellow")

# 🟡 Burası eksikti — şimdi tamam
def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightsService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

