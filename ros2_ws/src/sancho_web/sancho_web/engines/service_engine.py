import rclpy
from rclpy.node import Node
from rclpy.client import Client

from abc import ABC


class ServiceEngine(ABC):
    
    def __init__(self, node: Node):
        self.node = node

    def create_client(self, srv_type, srv_name):
        client = self.node.create_client(srv_type, srv_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'{srv_name} service not available, waiting again...')

        return client

    def call_service(self, client: Client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        return result