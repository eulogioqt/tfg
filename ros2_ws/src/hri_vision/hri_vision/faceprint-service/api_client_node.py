import json
import rclpy
from rclpy.node import Node
from hri_msgs.srv import GetString, Training

class APIClientNode(Node):
    def __init__(self):
        super().__init__('api_client_node')

        self.get_faceprint_client = self.create_client(GetString, 'recognition/get_faceprint')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get All service not available, waiting again...')

        self.training_client = self.create_client(Training, 'recognition/training')
        while not self.training_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Training service not available, waiting again...')

        self.get_logger().info("ROS Client Node initializated succesfully")

    def get_faceprint_request(self, name=None):
        get_all_request = GetString.Request()
        if name: get_all_request.args = json.dumps({ "name": name })

        future_get_all = self.get_faceprint_client.call_async(get_all_request)
        rclpy.spin_until_future_complete(self, future_get_all)
        result_get_all = future_get_all.result()

        return result_get_all.text
    
    def training_request(self, cmd_type_msg, args_msg):
        """Makes a training request to the training service.
        
        Args:
            cmd_type_msg (String): Training type (str) in ROS2 format (String).
            classified_msg (String): Class of the face.
            features_msg (float[]): Feature vector of the face.
            pos (int): Position of the vector with best distance (used to train).
        
        Returns:
            response (Training.srv): Result. -1 means something went wrong. 0 means everything is okay
                and in case of cmd_type = add_class, also means that the class wasn't already known. 1 means 
                that the class was already known, and means the same as 0 for cmd_type != add_class. Message...
        """

        training_request = Training.Request()
        training_request.cmd_type = cmd_type_msg
        training_request.args = args_msg

        future_training = self.node.training_client.call_async(training_request)
        rclpy.spin_until_future_complete(self.node, future_training)
        result_training = future_training.result()

        return result_training.result, result_training.message
