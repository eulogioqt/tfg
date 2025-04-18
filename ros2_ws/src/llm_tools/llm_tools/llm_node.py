import rclpy

from rclpy.node import Node
from llm_msgs.msg import LoadUnloadResult, ProviderModel
from llm_msgs.srv import GetModels, Prompt, Embedding, LoadModel, UnloadModel

from .providers.openai_provider import OpenAIProvider
from .providers.mistral_provider import MistralProvider
from .providers.phi_provider import PhiProvider
from .providers.qwen_provider import QwenProvider
from .providers.deepseek_provider import DeepSeekProvider
from .providers.gemini_provider import GeminiProvider
from .providers.sbert_provider import SBERTProvider
from .providers.e5_provider import E5Provider
from .providers.baai_provider import BAAIProvider

from .models import PROVIDER, MODELS

# IMPORTANTISIMO
# METER A FUTURO SISTEMA DE STREAMING EN TODOS LOS PROVIDERS O ALGO ASI, IMPLEMENTARLO CON UN ACTION Y DEMAS

# ver si hay mas modernos como el caso del qwen y eso de cada uno

# hacer que con parametros de ros tmb se pueda elegir que cargar y demas a parte del service
# poner algo para que si no cabe el modelo descargue los otros

class LLMNode(Node):

    PROVIDER_CLASS_MAP = {
        PROVIDER.OPENAI: OpenAIProvider,
        PROVIDER.MISTRAL: MistralProvider,
        PROVIDER.PHI: PhiProvider,
        PROVIDER.QWEN: QwenProvider,
        PROVIDER.DEEPSEEK: DeepSeekProvider,
        PROVIDER.GEMINI: GeminiProvider,
        PROVIDER.SBERT: SBERTProvider,
        PROVIDER.E5: E5Provider,
        PROVIDER.BAAI: BAAIProvider,
    }

    def __init__(self):
        super().__init__('llm')
        
        self.provider_map = {}

        self.get_all_srv = self.create_service(GetModels, 'llm_tools/get_all_models', self.handle_get_all_models)
        self.get_available_srv = self.create_service(GetModels, 'llm_tools/get_available_models', self.handle_get_available_models)
        self.prompt_srv = self.create_service(Prompt, 'llm_tools/prompt', self.handle_prompt)
        self.embedding_srv = self.create_service(Embedding, 'llm_tools/embedding', self.handle_embedding)
        self.load_model_srv = self.create_service(LoadModel, 'llm_tools/load_model', self.handle_load_model)
        self.unload_model_srv = self.create_service(UnloadModel, 'llm_tools/unload_model', self.handle_unload_model)

        self.get_logger().info("LLM Node initializated succesfully")

    def handle_get_all_models(self, request, response):
        provider_names = request.providers
        if not provider_names:
            provider_names = list(PROVIDER)

        self.get_logger().info(f"📖 Get Available Models service for providers: {provider_names}")
        
        response.llm_models = []
        response.embedding_models = []
        response.providers = []
        for provider_name in provider_names:
            if hasattr(MODELS.LLM, provider_name.upper()):
                llm_models = list(getattr(MODELS.LLM, provider_name.upper()))
                response.llm_models.append(ProviderModel(provider=provider_name, models=llm_models))

            if hasattr(MODELS.EMBEDDING, provider_name.upper()):
                embedding_models = list(getattr(MODELS.EMBEDDING, provider_name.upper()))
                response.embedding_models.append(ProviderModel(provider=provider_name, models=embedding_models))
                
            response.providers.append(provider_name)

        return response

    def handle_get_available_models(self, request, response):
        provider_names = request.providers
        if not provider_names:
            provider_names = list(PROVIDER)

        self.get_logger().info(f"📖 Get Available Models service for providers: {provider_names}")
        
        response.llm_models = []
        response.embedding_models = []
        response.providers = []
        for provider_name in provider_names:
            if provider_name in self.provider_map:
                provider = self.provider_map.get(provider_name)
                models = provider.get_active_models()

                llm_models = [m for m in models if m in list(getattr(MODELS.LLM, provider_name.upper(), []))]
                response.llm_models.append(ProviderModel(provider=provider_name, models=llm_models))

                embedding_models = [m for m in models if m in list(getattr(MODELS.EMBEDDING, provider_name.upper(), []))]
                response.embedding_models.append(ProviderModel(provider=provider_name, models=embedding_models))
                
                response.providers.append(provider_name)

        return response

    def handle_prompt(self, request, response):
        self.get_logger().info(f"📖 Prompt service for provider='{request.provider}', model='{request.model}'")

        try:
            provider_name, provider = self._get_provider(request.provider)            
            result, model_used = provider.prompt(
                model=request.model,
                prompt_system=request.prompt_system,
                messages_json=request.messages_json,
                user_input=request.user_input,
                parameters_json=request.parameters_json
            )

            response.response = result
            self._fill_response(response, True, "OK", provider_name, model_used)
            self.get_logger().info(f"✅ Prompt done using provider='{provider_name}', model='{model_used}'")
        except Exception as e:
            response.response = ""
            self._fill_response(response, False, str(e), request.provider, request.model)
            self.get_logger().info(f"❌ Prompt service failed: {str(e)}")

        return response

    def handle_embedding(self, request, response):
        self.get_logger().info(f"📖 Embedding service for provider='{request.provider}', model='{request.model}'")
        
        try:
            provider_name, provider = self._get_provider(request.provider)
            result, model_used = provider.embedding(
                model=request.model,
                user_input=request.user_input
            )

            response.embedding = result
            self._fill_response(response, True, "OK", provider_name, model_used)
            self.get_logger().info(f"✅ Embedding done using provider='{provider_name}', model='{model_used}'")
        except Exception as e:
            response.embedding = []
            self._fill_response(response, False, str(e), request.provider, request.model)
            self.get_logger().info(f"❌ Embedding service failed: {str(e)}")

        return response

    def handle_load_model(self, request, response):
        self.get_logger().info(f"📖 Load model service for items: {[{'provider': i.provider, 'models': i.models} for i in request.items]}'")

        response.results = []
        for item in request.items:
            result = LoadUnloadResult(provider=item.provider)
            try:
                provider = self._try_load_provider(item.provider, item.api_key)
                provider.load(item.models)

                self._fill_result(result, item.models, True, "Models loaded succesfully")
                self.get_logger().info(f"✅ Models {item.models} from provider {item.provider} loaded succesfully")
            except Exception as e:
                self._fill_result(result, [], False, f"Error loading models: {str(e)}")
                self.get_logger().info(f"❌ Load models service failed: {str(e)}")

            response.results.append(result)

        return response

    def handle_unload_model(self, request, response):
        self.get_logger().info(f"📖 Unload model service for items: {[{'provider': i.provider, 'models': i.models} for i in request.items]}'")
        
        response.results = []
        for item in request.items:
            result = LoadUnloadResult(provider=item.provider)
            provider = self.provider_map.get(item.provider)
            if provider:
                try:
                    provider.unload(item.models)
                    self._fill_result(result, True, "Models unloaded succesfully")
                    self.get_logger().info(f"✅ Models {item.models} from provider {item.provider} unloaded succesfully")
                except Exception as e:
                    self._fill_result(result, item.models, False, f"Error unloading models: {str(e)}")
                    self.get_logger().info(f"❌ Unload models service failed: {str(e)}")
            else:
                self._fill_result(result, [], False, f"Provider '{item.provider}' not found.")
                self.get_logger().info(f"❌ Load models service failed: Provider '{item.provider}' not found.")

            response.results.append(result)

        return response

    def _get_provider(self, requested_name):
        if requested_name in self.provider_map:
            return requested_name, self.provider_map[requested_name]

        if not self.provider_map:
            raise ValueError("No providers are loaded")

        fallback_name, fallback_provider = next(iter(self.provider_map.items()))
        self.get_logger().warn(f"[WARN] Provider '{requested_name}' not found. Using fallback '{fallback_name}' instead.")
        
        return fallback_name, fallback_provider

    def _try_load_provider(self, name, api_key=""):
        if name not in self.provider_map:
            if name not in self.PROVIDER_CLASS_MAP:
                raise ValueError(f"Provider '{name}' is not supported.")
            
            provider_class = self.PROVIDER_CLASS_MAP[name]
            self.provider_map[name] = provider_class(api_key=api_key) if api_key else provider_class()

        return self.provider_map[name]

    def _fill_response(self, response, success, message, provider, model):
        response.success = success
        response.message = message
        response.provider_used = provider
        response.model_used = model
    
    def _fill_result(self, result, models, success, message):
        result.models = models
        result.success = success
        result.message = message

def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    rclpy.spin(node)
    rclpy.shutdown()