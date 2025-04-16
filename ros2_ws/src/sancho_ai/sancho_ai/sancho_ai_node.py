import rclpy
from rclpy.node import Node

from hri_msgs.srv import SanchoPrompt

from .ais.factory import create_sancho_ai, AIType

class SanchoAINode(Node):

    def __init__(self, type):
        super().__init__("sancho_ai")

        self.sancho_ai = create_sancho_ai(type)
        self.prompt_serv = self.create_service(SanchoPrompt, "sancho_ai/prompt", self.prompt_service)

        self.get_logger().info("SanchoAI Node initializated succesfully")

    def prompt_service(self, request, response):
        response.text = self.sancho_ai.on_message(request.text)
        return response

def main(args=None):
    rclpy.init(args=args)

    node = SanchoAINode(AIType.CLASSIFICATION_TEMPLATES)

    rclpy.spin(node)
    rclpy.shutdown()