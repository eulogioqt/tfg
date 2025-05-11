import os
import json

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from dotenv import load_dotenv

from llm_msgs.srv import GetModels, Prompt, Embedding, LoadModel, UnloadModel
from llm_msgs.msg import LoadModel as LoadModelMsg, ProviderModel
from llm_tools.models import PROVIDER, MODELS


class TestNode(Node):

    def __init__(self):
        super().__init__("test")

        load_dotenv()

        self.cli_get_all = self.create_client(GetModels, 'llm_tools/get_all_models')
        self.cli_get_available = self.create_client(GetModels, 'llm_tools/get_available_models')
        self.cli_prompt = self.create_client(Prompt, 'llm_tools/prompt')
        self.cli_embedding = self.create_client(Embedding, 'llm_tools/embedding')
        self.cli_load = self.create_client(LoadModel, 'llm_tools/load_model')
        self.cli_unload = self.create_client(UnloadModel, 'llm_tools/unload_model')

        self.run_tests()

    def wait_for_service(self, client):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"Service {client.srv_name} not available.")
            return False
        return True

    def call_sync(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def run_tests(self):
        # 0. Mostrar todo
        self.show_all_models()

        # 1. Mostrar estado inicial
        self.get_logger().info("🔍 Estado inicial:")
        self.show_models()

        # 2. Cargar OpenAI y Gemini
        self.get_logger().info("🚀 Cargando OpenAI y Gemini...")
        #self.load_models(PROVIDER.OPENAI, ["gpt-3.5-turbo", "text-embedding-3-small"], api_key=os.environ.get("OPENAI_API_KEY"))
        self.load_models(PROVIDER.GEMINI, ["gemini-1.5-flash-latest"], api_key=os.environ.get("GEMINI_API_KEY"))
        self.show_models()

        # 3. Probar OpenAI y Gemini
        #self.test_provider(PROVIDER.OPENAI, ["gpt-3.5-turbo"], ["text-embedding-3-small"])
        self.test_provider(PROVIDER.GEMINI, ["gemini-1.5-flash-latest"], [])

        # 4. Probar todos los demás uno a uno
        skip = {PROVIDER.OPENAI, PROVIDER.GEMINI}
        for provider in PROVIDER:
            if provider in skip:
                continue

            llm_models = self.get_models_from_enum(MODELS.LLM, provider)
            emb_models = self.get_models_from_enum(MODELS.EMBEDDING, provider)

            self.get_logger().info(f"⚙️  Probando {provider}...")
            self.load_models(provider, llm_models + emb_models, **({"api_key": os.environ.get("HUGGING_FACE_API_KEY")} if provider == PROVIDER.MISTRAL else {}))
            self.test_provider(provider, llm_models, emb_models)
            self.unload_models(provider, llm_models + emb_models)
            self.get_logger().info("\n")

        self.get_logger().info("✅ ¡Pruebas finalizadas!")

    def load_models(self, provider, models, api_key=""):
        if not self.wait_for_service(self.cli_load):
            return
        req = LoadModel.Request()
        item = LoadModelMsg(provider=provider, models=models, api_key=api_key)
        req.items = [item]
        res = self.call_sync(self.cli_load, req)
        for r in res.results:
            self.get_logger().info(f"📦 {r.provider}: {r.message}")

    def unload_models(self, provider, models):
        if not self.wait_for_service(self.cli_unload):
            return
        req = UnloadModel.Request()
        item = ProviderModel(provider=provider, models=models)
        req.items = [item]
        res = self.call_sync(self.cli_unload, req)
        for r in res.results:
            self.get_logger().info(f"🧹 {r.provider}: {r.message}")

    def show_all_models(self):
        if not self.wait_for_service(self.cli_get_all):
            return
        req = GetModels.Request()
        res = self.call_sync(self.cli_get_all, req)

        self.get_logger().info(f"📖 Providers: {res.providers}")
        self.get_logger().info(f"📖 LLM Models:")
        for item in res.llm_models:
            self.get_logger().info(f"📖 {item.provider}: {item.models}")
        self.get_logger().info(f"📖 Embedding Models:")
        for item in res.embedding_models:
            self.get_logger().info(f"📖 {item.provider}: {item.models}")
        self.get_logger().info("\n")

    def show_models(self):
        if not self.wait_for_service(self.cli_get_available):
            return
        req = GetModels.Request()
        res = self.call_sync(self.cli_get_available, req)

        self.get_logger().info(f"📖 Providers: {res.providers}")
        self.get_logger().info(f"📖 LLM Models:")
        for item in res.llm_models:
            self.get_logger().info(f"📖 {item.provider}: {item.models}")
        self.get_logger().info(f"📖 Embedding Models:")
        for item in res.embedding_models:
            self.get_logger().info(f"📖 {item.provider}: {item.models}")
        self.get_logger().info("\n")

    def test_provider(self, provider, llm_models, emb_models):
        for model in llm_models:
            if self.wait_for_service(self.cli_prompt):
                req = Prompt.Request()
                req.provider = provider
                req.model = model
                req.prompt_system = "You are a test bot."
                req.messages_json = ""
                req.user_input = "Hello!"
                req.parameters_json = json.dumps({"temperature": 0.0, "max_tokens": 60})
                res = self.call_sync(self.cli_prompt, req)
                status = "✅" if res.success else "❌"
                self.get_logger().info(f"{status} Prompt [{provider}/{model}] Used [{res.provider_used}/{res.model_used}] → {res.response}")

        for model in emb_models:
            if self.wait_for_service(self.cli_embedding):
                req = Embedding.Request()
                req.provider = provider
                req.model = model
                req.user_input = "This is a test embedding input."
                res = self.call_sync(self.cli_embedding, req)
                status = "✅" if res.success else "❌"
                self.get_logger().info(f"{status} Embedding [{provider}/{model}] Used [{res.provider_used}/{res.model_used}] → dim={len(res.embedding)}")

    def get_models_from_enum(self, model_enum_group, provider):
        if not hasattr(model_enum_group, provider.name.upper()):
            return []
        enum_class = getattr(model_enum_group, provider.name.upper())
        return [str(m) for m in enum_class]


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
