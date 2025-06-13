"""TODO: Add module documentation."""
import json

from std_msgs.msg import String
from hri_msgs.srv import GetString as GetStringHRI, Training
from rumi_msgs.srv import GetString as GetStringRUMI

from .service_engine import ServiceEngine


class HRIEngine(ServiceEngine):
"""TODO: Describe class."""
    def __init__(self, node):
    """TODO: Describe __init__.
Args:
    node (:obj:`Any`): TODO.
"""
        super().__init__(node)

        self.get_sessions_cli = self.create_client(GetStringRUMI, 'rumi/sessions/get', wait=False)
        self.get_sessions_summary_cli = self.create_client(GetStringRUMI, 'rumi/sessions/get_summary', wait=False)

        self.get_faceprint_cli = self.create_client(GetStringHRI, 'recognition/get_faceprint', wait=False)
        self.actual_people_cli = self.create_client(GetStringHRI, "logic/get/actual_people", wait=False)
        self.get_last_frame_cli = self.create_client(GetStringHRI, "logic/get/last_frame", wait=False)
        self.training_cli = self.create_client(Training, 'recognition/training', wait=False)

        self.node.get_logger().info("HRI Engine initializated succesfully")

    # Action
    def delete_request(self, id):
    """TODO: Describe delete_request.
Args:
    id (:obj:`Any`): TODO.
"""
        req = Training.Request()
        req.cmd_type = String(data="delete_class")
        req.args = String(data=json.dumps({ "class_id": id }))

        result = self.call_service(self.training_cli, req)
        if result is None:
            return -1
    
        return result.result
    
    def rename_request(self, id, new_name):
    """TODO: Describe rename_request.
Args:
    id (:obj:`Any`): TODO.
    new_name (:obj:`Any`): TODO.
"""
        req = Training.Request()
        req.cmd_type = String(data="rename_class")
        req.args = String(data=json.dumps({ "class_id": id, "new_name": new_name }))

        result = self.call_service(self.training_cli, req)
        if result is None:
            return -1
    
        return result.result
    
    # Info
    def get_faceprint_request(self, args_msg=""):
    """TODO: Describe get_faceprint_request.
Args:
    args_msg (:obj:`Any`): TODO.
"""
        req = GetStringHRI.Request()
        req.args = args_msg

        result = self.call_service(self.get_faceprint_cli, req)
        if result is None:
            return "[]"
        
        return result.text

    def get_actual_people_request(self):
    """TODO: Describe get_actual_people_request.
"""
        req = GetStringHRI.Request()

        result = self.call_service(self.actual_people_cli, req)
        if result is None:
            return "{}"
        
        return result.text

    def get_sessions_request(self, args_msg=""):
    """TODO: Describe get_sessions_request.
Args:
    args_msg (:obj:`Any`): TODO.
"""
        req = GetStringRUMI.Request()
        req.args = args_msg

        result = self.call_service(self.get_sessions_cli, req)
        if result is None:
            return "[]"

        return result.text
    
    def get_sessions_summary_request(self, args_msg=""):
    """TODO: Describe get_sessions_summary_request.
Args:
    args_msg (:obj:`Any`): TODO.
"""
        req = GetStringRUMI.Request()
        req.args = args_msg

        result = self.call_service(self.get_sessions_summary_cli, req)
        if result is None:
            return "[]"

        return result.text

    def get_last_frame_request(self):
    """TODO: Describe get_last_frame_request.
"""
        req = GetStringHRI.Request()

        result = self.call_service(self.get_last_frame_cli, req)
        if result is None:
            return ""
        
        return result.text
