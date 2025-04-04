import rclpy
from rclpy.node import Node

from hri_msgs.srv import SanchoPrompt

class SanchoAINode(Node):

    def __init__(self, sancho_ai: "SanchoAI"):
        super().__init__("sancho_ai")

        self.sancho_ai = sancho_ai
        self.prompt_serv = self.create_service(SanchoPrompt, "sancho_ai/prompt", self.prompt_service)

        self.get_logger().info("SanchoAI Node initializated succesfully")

    def prompt_service(self, request, response):
        response.id = request.id
        response.text = self.sancho_ai.on_message(request.text)
        return response

class SanchoAI:
    
    def __init__(self):
        self.node = SanchoAINode(self)

    def spin(self):
        while True:
            rclpy.spin_once(self.node)

    def on_message(self, message):
        return message[::-1]


def main(args=None):
    rclpy.init(args=args)

    sancho_ai = SanchoAI()

    sancho_ai.spin()
    rclpy.shutdown()