import rclpy
from rclpy.node import Node

from hri_msgs.srv import SanchoPrompt

from .ais.factory import create_sancho_ai, AIType

# meter comandos de dime cual es la ultima persona que has visto
# de que diga cuanta gente conoce
# lo ideal seguramente sea en el contexto meterle toda esta info si son demasiados comandos y dejar comando solo para lo que haga acciones
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