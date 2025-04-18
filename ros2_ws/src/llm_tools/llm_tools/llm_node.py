import os
import json
import rclpy

from rclpy.node import Node
from llm_msgs.srv import Prompt, Embedding

from dotenv import load_dotenv

from .providers.base_provider import BaseProvider
from .providers.openai_provider import OpenAIProvider
from .providers.mistral_provider import MistralProvider
from .providers.phi_provider import PhiProvider
from .providers.qwen_provider import QwenProvider
from .providers.deepseek_provider import DeepSeekProvider
from .providers.gemini_provider import GeminiProvider
from .providers.e5_provider import E5Provider
from .providers.sbert_provider import SBERTProvider
from .providers.baai_provider import BAAIProvider

from .models import MODELS, PROVIDER

# IMPORTANTISIMO
# METER A FUTURO SISTEMA DE STREAMING EN TODOS LOS PROVIDERS O ALGO ASI, IMPLEMENTARLO CON UN ACTION Y DEMAS

# mañana, probar gemini
# probar los de embeddings
# ver si hay mas modernos como el caso del qwen y eso de cada uno
# ver si meter otros y si quitar phi y eso
# hacer un service para elegir que se va a cargar y que no, que si no es un folloncete

# refactor si necesario
# pyl

# refactorizar para que no pida token si no es necesario en los modelos de hugging face
# forma de saber cuales tienen embeddings y cuales tienen prompt y cuales los dos

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm')

        load_dotenv()
        self.provider_map = self._load_providers()

        self.prompt_srv = self.create_service(Prompt, 'llm_tools/prompt', self.handle_prompt)
        self.embedding_srv = self.create_service(Embedding, 'llm_tools/embedding', self.handle_embedding)

        self.get_logger().info(f"LLM Node initializated succesfully with providers: {list(self.provider_map.keys())}")

    def handle_prompt(self, request, response):
        provider = self.provider_map.values()[0] #self.get_provider(request.provider.lower())
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
        provider = self.get_provider(request.provider.lower())
        result = provider.embedding(
            model=request.model,
            user_input=request.user_input
        )
        response.embedding = result

        return response

    def get_provider(self, provider_name: str) -> BaseProvider:
        provider = self.provider_map.get(provider_name)
        if not provider:
            fallback = list(self.provider_map.keys())[0]
            provider = self.provider_map[fallback]
            self.get_logger().warn(f"Provider '{provider_name}' not available. Using '{fallback}' as fallback.")

        return provider

    def _load_providers(self):
        providers = {}

        openai_key = os.getenv("OPENAI_API_KEY")
        hugging_face_key = os.getenv("HUGGING_FACE_API_KEY")
        gemini_key = os.getenv("GEMINI_API_KEY")
     
        prompt_system = "Solo puedes responder en formato json SOLO EL JSON SIN NADA MAS, PARA PARSEAR. Tienes que coger el texto del usuario y ponerlo en formato json como quieras siempre."
        user_input = "Hola me llamo luis suarez, y tu?"
        temperature = 0.0
        max_tokens = 100

        def test(provider: BaseProvider, models=None):
            print(f"\n\n🧪 Testing {provider.__class__.__name__}")
            
            if models:
                print(f"📦 Loading models: {[model.value for model in models]}")
                provider.load(models)
            
            try:
                print("🧠 Testing prompt()...")
                result = provider.prompt(
                    user_input=user_input,
                    model=None,
                    prompt_system=prompt_system,
                    messages_json=None,
                    parameters_json=json.dumps({
                        "temperature": temperature,
                        "max_tokens": max_tokens
                    })
                )
                print("📤 Prompt result:", result)
            except Exception as e:
                print(f"⚠️ Prompt not supported: {str(e)}")

            try:
                print("🔎 Testing embedding()...")
                result = provider.embedding(user_input=user_input, model=None)
                print("📏 Embedding length:", len(result))
            except Exception as e:
                print(f"⚠️ Embedding not supported: {str(e)}")

            print("🧹 Unloading models...")
            provider.unload()

        #providers["openai"] = OpenAIProvider(api_key=openai_key)
        #test(providers["openai"])

        providers["mistral"] = MistralProvider(api_key=hugging_face_key)
        test(providers["mistral"], [MODELS.LLM.MISTRAL.MISTRAL_7B])

        providers["phi"] = PhiProvider()
        test(providers["phi"], [MODELS.LLM.PHI.PHI_2])

        providers["qwen"] = QwenProvider()
        test(providers["qwen"], [MODELS.LLM.QWEN.QWEN_7B, MODELS.EMBEDDING.QWEN.QWEN_EMBED])

        providers["deepseek"] = DeepSeekProvider()
        test(providers["deepseek"], [MODELS.LLM.DEEPSEEK.DEEPSEEK_CHAT, MODELS.EMBEDDING.DEEPSEEK.DEEP_EMBED])

        providers["sbert"] = SBERTProvider()
        test(providers["sbert"],  [MODELS.EMBEDDING.SBERT.MINI_LM_L6_V2])

        providers["e5"] = E5Provider()
        test(providers["e5"],  [MODELS.EMBEDDING.E5.E5_LARGE_V2])

        providers["baai"] = BAAIProvider()
        test(providers["baai"],  [MODELS.EMBEDDING.BAAI.BGE_BASE_EN])

        providers["gemini"] = GeminiProvider(api_key=gemini_key)
        test(providers["gemini"])

        if not providers:
            self.get_logger().error("Couldn't load any LLM provider. Closing node.")
            raise RuntimeError("No LLM providers available")

        return providers

def main(args=None):
    rclpy.init(args=args)

    llm_node = LLMNode()

    rclpy.spin(llm_node)
    rclpy.shutdown()
