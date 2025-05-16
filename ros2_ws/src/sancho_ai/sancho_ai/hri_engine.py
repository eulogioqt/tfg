import json

from std_msgs.msg import String
from hri_msgs.srv import GetString, Training

from .service_engine import ServiceEngine


class HRIEngine(ServiceEngine):
    def __init__(self, node):
        super().__init__(node)

        self.get_faceprint_cli = self.create_client(GetString, 'recognition/get_faceprint', wait=False)
        self.actual_people_cli = self.create_client(GetString, "logic/get/actual_people", wait=False)
        self.training_cli = self.create_client(Training, 'recognition/training', wait=False)

        self.node.get_logger().info("HRI Engine initializated succesfully")

    # Action
    def delete_request(self, id):
        req = Training.Request()
        req.cmd_type = String(data="delete_class")
        req.args = String(data=json.dumps({ "class_id": id }))

        result = self.call_service(self.training_cli, req)
        if result is None:
            return -1
    
        return result.result
    
    def rename_request(self, id, new_name):
        req = Training.Request()
        req.cmd_type = String(data="rename_class")
        req.args = String(data=json.dumps({ "class_id": id, "new_name": new_name }))

        result = self.call_service(self.training_cli, req)
        if result is None:
            return -1
    
        return result.result

    def take_picture_request(self):
        return ""

    # Info
    def get_faceprint_request(self, args_msg=""):
        req = GetString.Request()
        req.args = args_msg

        result = self.call_service(self.get_faceprint_cli, req)
        if result is None:
            return "[]"
        
        return result.text

    def get_actual_people_request(self):
        req = GetString.Request()

        result = self.call_service(self.actual_people_cli, req)
        if result is None:
            return "{}"
        
        return result.text
