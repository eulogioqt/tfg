import rclpy
from rclpy.node import Node

from hri_msgs.srv import SanchoPrompt

from .ais.ai import AI

from .ais.simple_ai import SimpleAI
from .ais.classification_templates_ai import ClassificationTemplatesAI

class SanchoAINode(Node):

    def __init__(self, sancho_ai: AI):
        super().__init__("sancho_ai")

        self.sancho_ai = sancho_ai
        self.prompt_serv = self.create_service(SanchoPrompt, "sancho_ai/prompt", self.prompt_service)

        self.get_logger().info("SanchoAI Node initializated succesfully")

    def prompt_service(self, request, response):
        response.text = self.sancho_ai.on_message(request.text)
        return response

def main(args=None):
    rclpy.init(args=args)

    sancho_ai = ClassificationTemplatesAI()

    sancho_ai.spin()
    rclpy.shutdown()