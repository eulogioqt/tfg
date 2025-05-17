import json
import rclpy
from rclpy.node import Node

from hri_msgs.srv import SanchoPrompt

from .log_manager import LogManager
from .ais import create_sancho_ai, AIType


class SanchoAINode(Node):

    def __init__(self, type):
        super().__init__("sancho_ai")

        self.type = type
        self.sancho_ai = create_sancho_ai(self.type)
        self.prompt_serv = self.create_service(SanchoPrompt, "sancho_ai/prompt", self.prompt_service)

        self.chats = {}

        LogManager.set_node(self)
        self.get_logger().info("SanchoAI Node initializated successfully")

    def prompt_service(self, request, response):
        chat_history = self.chats.get(request.user, [])

        ai_response, intent, arguments, provider, model = self.sancho_ai.on_message(request.text, chat_history)

        chat_history.append({"role": "user", "content": request.text})
        chat_history.append({"role": "assistant", "content": ai_response})

        self.chats[request.user] = chat_history[-10:]  # 5 turnos

        response.text = ai_response
        response.method = self.type
        response.intent = intent
        response.args_json = json.dumps(arguments)
        response.provider = provider
        response.model = model

        return response


def main(args=None):
    rclpy.init(args=args)

    node = SanchoAINode(AIType.LLM_CLASSIFIER_GENERATOR)

    rclpy.spin(node)
    rclpy.shutdown()