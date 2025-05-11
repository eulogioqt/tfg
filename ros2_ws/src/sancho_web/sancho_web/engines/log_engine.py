from hri_msgs.srv import CreateLog, GetString
from hri_vision.database.system_database import CONSTANTS

from .service_engine import ServiceEngine


class LogEngine(ServiceEngine):
    def __init__(self, node):
        super().__init__(node)

        self.create_log_cli = self.create_client(CreateLog, 'logic/create_log')
        self.get_logs_cli = self.create_client(GetString, 'logic/get/logs')

        self.node.get_logger().info("Log Engine initializated successfully")

    def create_log_request(self, action, faceprint_id):
        req = CreateLog.Request()
        req.action = action
        req.faceprint_id = faceprint_id
        req.origin = CONSTANTS.ORIGIN_WEB

        result = self.call_service(self.create_log_cli, req)

        return result.success, result.message
    
    def get_logs_request(self, args_msg=""):
        req = GetString.Request()
        req.args = args_msg

        result = self.call_service(self.get_logs_cli, req)

        return result.text