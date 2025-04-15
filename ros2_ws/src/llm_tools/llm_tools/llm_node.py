import os
import rclpy

from rclpy.node import Node
from llm_msgs.srv import Prompt, Embedding

from dotenv import load_dotenv

from .api.openai_provider import OpenAIProvider

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        self._load_dotenv()
        self.provider_map = self._load_providers()

        self.prompt_srv = self.create_service(Prompt, 'llm_tools/prompt', self.handle_prompt)
        self.embedding_srv = self.create_service(Embedding, 'llm_tools/embedding', self.handle_embedding)

        self.get_logger().info(f"LLM Node initializated succesfully with providers: {list(self.provider_map.keys())}")

    def handle_prompt(self, request, response):
        provider = self.get_provider(request.provider.lower())
        result = provider.prompt(
            user_input=request.user_input,
            model=request.model,
            prompt_system=request.prompt_system,
            messages_json=request.messages_json,
            parameters_json=request.parameters_json
        )
        response.response = result

        return response

    def handle_embedding(self, request, response):
        provider = self.get_provider(request.provider.lower())
        result = provider.embedding(
            user_input=request.user_input,
            model=request.model
        )
        response.embedding = result

        return response

    def get_provider(self, provider_name: str):
        provider = self.provider_map.get(provider_name)
        if not provider:
            fallback = list(self.provider_map.keys())[0]
            provider = self.provider_map[fallback]
            self.get_logger().warn(f"Provider '{provider_name}' not available. Using '{fallback}' as fallback.")

        return provider

    def _load_providers(self):
        providers = {}

        openai_key = os.getenv("OPENAI_API_KEY")

        if openai_key: providers["openai"] = OpenAIProvider(api_key=openai_key)
        else: self.get_logger().warn("OPENAI_API_KEY not defined")

        if not providers:
            self.get_logger().error("Couldn't load any LLM provider. Closing node.")
            raise RuntimeError("No LLM providers available")

        return providers

    def _load_dotenv(self):
        load_dotenv()

def main(args=None):
    rclpy.init(args=args)

    llm_node = LLMNode()

    rclpy.spin(llm_node)
    rclpy.shutdown()
