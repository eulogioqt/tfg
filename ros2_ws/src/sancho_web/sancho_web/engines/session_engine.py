"""TODO: Add module documentation."""
from rumi_msgs.srv import GetString

from .service_engine import ServiceEngine


class SessionEngine(ServiceEngine):
"""TODO: Describe class."""
    def __init__(self, node):
    """TODO: Describe __init__.
Args:
    node (:obj:`Any`): TODO.
"""
        super().__init__(node)

        self.get_sessions_cli = self.create_client(GetString, 'rumi/sessions/get')
        self.get_sessions_summary_cli = self.create_client(GetString, 'rumi/sessions/get_summary')

        self.node.get_logger().info("Session Engine initializated successfully")

    def get_sessions_request(self, args_msg=""):
    """TODO: Describe get_sessions_request.
Args:
    args_msg (:obj:`Any`): TODO.
"""
        req = GetString.Request()
        req.args = args_msg

        result = self.call_service(self.get_sessions_cli, req)

        return result.text
    
    def get_sessions_summary_request(self):
    """TODO: Describe get_sessions_summary_request.
"""
        req = GetString.Request()

        result = self.call_service(self.get_sessions_summary_cli, req)

        return result.text  
