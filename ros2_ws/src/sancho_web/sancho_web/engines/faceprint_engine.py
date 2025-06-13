"""TODO: Add module documentation."""
from hri_msgs.srv import Detection, Recognition, Training, GetString

from hri_vision.hri_bridge import HRIBridge

from .service_engine import ServiceEngine


class FaceprintEngine(ServiceEngine):
"""TODO: Describe class."""
    def __init__(self, node):
    """TODO: Describe __init__.
Args:
    node (:obj:`Any`): TODO.
"""
        super().__init__(node)

        self.get_faceprint_cli = self.create_client(GetString, 'recognition/get_faceprint')
        self.detection_cli = self.create_client(Detection, 'detection')
        self.recognition_cli = self.create_client(Recognition, 'recognition')
        self.training_cli = self.create_client(Training, 'recognition/training')

        self.br = HRIBridge()

        self.node.get_logger().info("Faceprint Engine initializated successfully")

    def get_faceprint_request(self, args_msg=""):
    """TODO: Describe get_faceprint_request.
Args:
    args_msg (:obj:`Any`): TODO.
"""
        req = GetString.Request()
        req.args = args_msg

        result = self.call_service(self.get_faceprint_cli, req)

        return result.text

    def detection_request(self, frame_msg):
    """TODO: Describe detection_request.
Args:
    frame_msg (:obj:`Any`): TODO.
"""
        req = Detection.Request()
        req.frame = frame_msg

        result = self.call_service(self.detection_cli, req)

        return result.positions, result.scores

    def recognition_request(self, frame_msg, position_msg, score_msg):
    """TODO: Describe recognition_request.
Args:
    frame_msg (:obj:`Any`): TODO.
    position_msg (:obj:`Any`): TODO.
    score_msg (:obj:`Any`): TODO.
"""
        req = Recognition.Request()
        req.frame = frame_msg
        req.position = position_msg
        req.score = score_msg

        result = self.call_service(self.recognition_cli, req)

        return result.face_aligned, result.features, result.classified_id, \
                result.classified_name, result.distance, result.pos, result.face_updated

    def training_request(self, cmd_type_msg, args_msg):
    """TODO: Describe training_request.
Args:
    cmd_type_msg (:obj:`Any`): TODO.
    args_msg (:obj:`Any`): TODO.
"""
        req = Training.Request()
        req.cmd_type = cmd_type_msg
        req.args = args_msg
        req.origin = Training.Request.ORIGIN_WEB

        result = self.call_service(self.training_cli, req)

        return result.result, result.message.data
