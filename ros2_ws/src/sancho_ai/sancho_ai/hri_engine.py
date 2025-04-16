from .service_engine import ServiceEngine

from hri_msgs.srv import GetString, Recognition


class HRIEngine(ServiceEngine):
    def __init__(self, node):
        super().__init__(node)

        self.actual_people_cli = self.create_client(GetString, "logic/get/actual_people"),

        self.node.get_logger().info("HRI Engine initializated succesfully")

    def get_actual_people_request(self):
        req = GetString.Request()

        result = self.call_service(self.actual_people_cli, req)

        return result.text