import os

from .service_engine import ServiceEngine

from dotenv import load_dotenv

from llm_msgs.srv import Prompt, LoadModel
from llm_msgs.msg import LoadModel as LoadModelMsg

from llm_tools.models import PROVIDER, MODELS # es buena idea esto? arreglar ya que no completa


class LLMEngine(ServiceEngine):
    def __init__(self, node):
        super().__init__(node)

        load_dotenv()

        self.load_model_cli = self.create_client(LoadModel, "llm_tools/load_model")
        self.prompt_cli = self.create_client(Prompt, "llm_tools/prompt")

        #self._load_models([
        #    [PROVIDER.GEMINI, [], os.environ.get("GEMINI_API_KEY")],
        #    [PROVIDER.DEEPSEEK, [MODELS.LLM.DEEPSEEK.DEEPSEEK_CHAT], ""]
        #])

        self.node.get_logger().info("Prompt Engine initializated succesfully")
        
    def load_model_request(self, items):
        req = LoadModel.Request()
        req.items = []

        for item in items:
            provider, models, api_key = item
            req.items.append(LoadModelMsg(provider=provider, models=models, api_key=api_key))

        result = self.call_service(self.load_model_cli, req)

        return result.results

    def prompt_request(self, provider="", model="", prompt_system="", messages_json="", user_input="", parameters_json=""):
        req = Prompt.Request()
        req.provider = provider
        req.model = model
        req.prompt_system = prompt_system
        req.messages_json = messages_json
        req.user_input = user_input
        req.parameters_json = parameters_json

        result = self.call_service(self.prompt_cli, req)

        return result.response, result.provider_used, result.model_used, result.message, result.success
    
    def _load_models(self, items):
        results = self.load_model_request(items)
        for result in results:
            if result.success:
                self.node.get_logger().info(f"Loaded from provider {result.provider} models {result.models}: {result.message}")
            else:
                self.node.get_logger().info(f"Could not load from provider {result.provider} models {result.models}: {result.message}")
