"""TODO: Add module documentation."""
from hri_msgs.srv import GetString

from .service_engine import ServiceEngine


class LogEngine(ServiceEngine):
"""TODO: Describe class."""
    def __init__(self, node):
    """TODO: Describe __init__.
Args:
    node (:obj:`Any`): TODO.
"""
        super().__init__(node)

        self.get_logs_cli = self.create_client(GetString, 'logs/get')

        self.node.get_logger().info("Log Engine initializated successfully")

    def get_logs_request(self, args_msg=""):
    """TODO: Describe get_logs_request.
Args:
    args_msg (:obj:`Any`): TODO.
"""
        req = GetString.Request()
        req.args = args_msg

        result = self.call_service(self.get_logs_cli, req)

        return result.text
