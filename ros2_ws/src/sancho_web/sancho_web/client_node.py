import rclpy

from rclpy.node import Node

from hri_msgs.srv import Detection, Recognition, Training, GetString
from hri_vision.hri_bridge import HRIBridge


class ClientNode(Node):
    def __init__(self):
        super().__init__('api_client_node')

        self.get_faceprint_client = self.create_client(GetString, 'recognition/get_faceprint')
        while not self.get_faceprint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get All service not available, waiting again...')

        self.detection_client = self.create_client(Detection, 'detection')
        while not self.detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Detection service not available, waiting again...')
        
        self.recognition_client = self.create_client(Recognition, 'recognition')
        while not self.recognition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Recognition service not available, waiting again...')

        self.training_client = self.create_client(Training, 'recognition/training')
        while not self.training_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Training service not available, waiting again...')

        self.br = HRIBridge()
        self.get_logger().info("ROS Client Node initializated succesfully")

    def get_faceprint_request(self, args_msg=""):
        get_all_request = GetString.Request()
        get_all_request.args = args_msg

        future_get_all = self.get_faceprint_client.call_async(get_all_request)
        rclpy.spin_until_future_complete(self, future_get_all)
        result_get_all = future_get_all.result()

        return result_get_all.text

    # Demasiada replica de hri logic, no?
    # Ademas cuando lo haga con el llm lo mismo, mas replica aun porque por ahi va por websocket a sancho ai....
    def detection_request(self, frame_msg):
        detection_request = Detection.Request()
        detection_request.frame = frame_msg

        future_detection = self.detection_client.call_async(detection_request)
        rclpy.spin_until_future_complete(self, future_detection)
        result_detection = future_detection.result()

        return result_detection.positions, result_detection.scores

    def recognition_request(self, frame_msg, position_msg, score_msg):
        recognition_request = Recognition.Request()
        recognition_request.frame = frame_msg
        recognition_request.position = position_msg
        recognition_request.score = score_msg

        future_recognition = self.recognition_client.call_async(recognition_request)
        rclpy.spin_until_future_complete(self, future_recognition)
        result_recognition = future_recognition.result()

        return (result_recognition.face_aligned, result_recognition.features,
                result_recognition.classified, result_recognition.distance, result_recognition.pos)    

    def training_request(self, cmd_type_msg, args_msg):
        training_request = Training.Request()
        training_request.cmd_type = cmd_type_msg
        training_request.args = args_msg
        training_request.origin = Training.Request.ORIGIN_WEB

        future_training = self.training_client.call_async(training_request)
        rclpy.spin_until_future_complete(self, future_training)
        result_training = future_training.result()

        return result_training.result, result_training.message.data
