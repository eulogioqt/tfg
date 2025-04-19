import json
import rclpy
import uvicorn

from rclpy.node import Node
from hri_msgs.srv import GetString, Training

from .faceprint_service.app import app
from .faceprint_service.v1 import set_api_node


class APIClientNode(Node):
    def __init__(self):
        super().__init__('api_client_node')

        self.get_faceprint_client = self.create_client(GetString, 'recognition/get_faceprint')
        while not self.get_faceprint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get All service not available, waiting again...')

        self.training_client = self.create_client(Training, 'recognition/training')
        while not self.training_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Training service not available, waiting again...')

        self.get_logger().info("ROS Client Node initializated succesfully")

    def spin(self):
        set_api_node(self)
        uvicorn.run(app, host="localhost", port=7654)

    def get_faceprint_request(self, args_msg=""):
        get_all_request = GetString.Request()
        get_all_request.args = args_msg

        future_get_all = self.get_faceprint_client.call_async(get_all_request)
        rclpy.spin_until_future_complete(self, future_get_all)
        result_get_all = future_get_all.result()

        return json.loads(result_get_all.text)
    
    def training_request(self, cmd_type_msg, args_msg):
        training_request = Training.Request()
        training_request.cmd_type = cmd_type_msg
        training_request.args = args_msg
        training_request.origin = Training.Request.ORIGIN_WEB

        future_training = self.training_client.call_async(training_request)
        rclpy.spin_until_future_complete(self, future_training)
        result_training = future_training.result()

        return result_training.result, result_training.message.data

def main(args=None):
    rclpy.init(args=args)

    api_node = APIClientNode()

    api_node.spin()
    rclpy.shutdown()

