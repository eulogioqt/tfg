import json
import rclpy
from rclpy.node import Node
from hri_msgs.srv import GetString 

class RosClientNode(Node):
    def __init__(self):
        super().__init__('api_client_node')
        self.cli = self.create_client(GetString, 'recognition/get_people')
        self.get_logger().info("Esperando al servicio 'get_all_faceprints'...")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Esperando al servicio...')
        self.get_logger().info("Nodo bien")

    def pedir_faceprints(self):
        req = GetString.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return json.loads(response.text.data)
