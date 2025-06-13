"""TODO: Add module documentation."""
import rclpy
from rclpy.node import Node
from rclpy.client import Client

from abc import ABC

from hri_msgs.msg import Log

from sancho_web.database.system_database import CONSTANTS


class ServiceEngine(ABC):
    
"""TODO: Describe class."""
    def __init__(self, node: Node):
    """TODO: Describe __init__.
Args:
    node (:obj:`Any`): TODO.
"""
        self.node = node
        self.clients = {} 

        self.publisher_log = self.node.create_publisher(Log, 'logs/add', 10)

    def create_log(self, action, target, message="", metadata_json="", level=CONSTANTS.LEVEL.INFO):
    """TODO: Describe create_log.
Args:
    action (:obj:`Any`): TODO.
    target (:obj:`Any`): TODO.
    message (:obj:`Any`): TODO.
    metadata_json (:obj:`Any`): TODO.
    level (:obj:`Any`): TODO.
"""
        self.publisher_log.publish(Log(
            level=level,
            origin=CONSTANTS.ORIGIN.WEB,
            actor="An user",
            action=action,
            target=target,
            message=message,
            metadata_json=metadata_json
        ))

    def create_client(self, srv_type, srv_name):
    """TODO: Describe create_client.
Args:
    srv_type (:obj:`Any`): TODO.
    srv_name (:obj:`Any`): TODO.
"""
        if srv_name in self.clients:
            return self.clients[srv_name]

        client = self.node.create_client(srv_type, srv_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'{srv_name} service not available, waiting again...')

        self.clients[srv_name] = client
        
        return client

    def call_service(self, client: Client, request):
    """TODO: Describe call_service.
Args:
    client (:obj:`Any`): TODO.
    request (:obj:`Any`): TODO.
"""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        return result
