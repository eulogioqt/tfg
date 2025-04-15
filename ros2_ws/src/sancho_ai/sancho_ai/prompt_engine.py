import rclpy

from rclpy.node import Node
from llm_msgs.srv import Prompt

class PromptEngine:

    def __init__(self, node: Node):
        self.node = node

        self.prompt_cli = self.node.create_client(Prompt, "llm_tools/prompt")
        if not self.prompt_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('LLM Tools Prompt service not available, waiting again...')
    
    def prompt_request(self, provider="openai", model="gpt-3.5-turbo", prompt_system="", messages_json="", user_input="", parameters_json=""):
        prompt_request = Prompt.Request()
        prompt_request.provider = provider
        prompt_request.model = model
        prompt_request.prompt_system = prompt_system
        prompt_request.messages_json = messages_json
        prompt_request.user_input = user_input
        prompt_request.parameters_json = parameters_json

        future_prompt = self.prompt_cli.call_async(prompt_request)
        rclpy.spin_until_future_complete(self.node, future_prompt)
        result_prompt = future_prompt.result()

        return result_prompt.response
