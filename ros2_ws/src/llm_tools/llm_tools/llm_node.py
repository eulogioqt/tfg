import os
import rclpy

from rclpy.node import Node
from llm_msgs.srv import Prompt, Embedding, LoadModel, UnloadModel

from dotenv import load_dotenv

from .providers.base_provider import BaseProvider
from .models import MODELS, PROVIDER, PROVIDER_CLASS_MAP

# IMPORTANTISIMO
# METER A FUTURO SISTEMA DE STREAMING EN TODOS LOS PROVIDERS O ALGO ASI, IMPLEMENTARLO CON UN ACTION Y DEMAS

# ver si hay mas modernos como el caso del qwen y eso de cada uno
# forma de saber cuales tienen embeddings y cuales tienen prompt y cuales los dos

# hacer que con parametros de ros tmb se pueda elegir que cargar y demas a parte del service
# poner algo para que si no cabe el modelo descargue los otros

class LLMNode(Node):

    def __init__(self):
        super().__init__('llm')
        
        self.provider_map = {}

        self.prompt_srv = self.create_service(Prompt, 'llm_tools/prompt', self.handle_prompt)
        self.embedding_srv = self.create_service(Embedding, 'llm_tools/embedding', self.handle_embedding)
        self.load_model_srv = self.create_service(LoadModel, 'llm_tools/load_model', self.handle_load_model)
        self.unload_model_srv = self.create_service(UnloadModel, 'llm_tools/unload_model', self.handle_unload_model)

        self.get_logger().info(f"LLM Node initializated succesfully")

    def handle_prompt(self, request, response):
        provider = self.get_provider(request.provider)
        result = provider.prompt(
            model=request.model,
            prompt_system=request.prompt_system,
            messages_json=request.messages_json,
            user_input=request.user_input,
            parameters_json=request.parameters_json
        )
        response.response = result

        return response

    def handle_embedding(self, request, response):
        provider = self.get_provider(request.provider)
        result = provider.embedding(
            model=request.model,
            user_input=request.user_input
        )
        response.embedding = result

        return response

    def handle_load_model(self, request, response):
        for item in request.items:
            provider = self.provider_map.get(item.provider)
            if not provider:
                provider_class = PROVIDER_CLASS_MAP[item.provider]
                provider = provider_class(**({"api_key": item.api_key} if item.api_key else {}))
            
            self.provider_map[item.provider] = provider.load(item.models)

    def handle_unload_model(self, request, response):
        for item in request.items:
            provider = self.provider_map.get(request.provider)
            if provider:
                provider.unload(request.models)

    def get_provider(self, provider_name: str) -> BaseProvider:
        provider = self.provider_map.get(provider_name)
        if not provider:
            fallback = list(self.provider_map.keys())[0]
            provider = self.provider_map[fallback]
            self.get_logger().warn(f"Provider '{provider_name}' not available. Using '{fallback}' as fallback.")

        return provider
    
    def load_api_keys():
        load_dotenv()

        return {
            PROVIDER.OPENAI: os.getenv("OPENAI_API_KEY"),
            PROVIDER.GEMINI: os.getenv("GEMINI_API_KEY"),
            PROVIDER.MISTRAL: os.getenv("HUGGING_FACE_API_KEY")
        }

def main(args=None):
    rclpy.init(args=args)

    llm_node = LLMNode()

    rclpy.spin(llm_node)
    rclpy.shutdown()
