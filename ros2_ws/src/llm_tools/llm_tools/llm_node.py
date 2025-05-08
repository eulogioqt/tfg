import rclpy
import importlib

from rclpy.node import Node
from llm_msgs.msg import LoadUnloadResult, ProviderModel
from llm_msgs.srv import GetModels, Prompt, Embedding, LoadModel, UnloadModel, GetActiveModels, SetActiveModel

from .models import PROVIDER, MODELS
from .providers.base_provider import BaseProvider


class LLMNode(Node):

    PROVIDER_CLASS_MAP = { # Poner esto mas cool, solo con el nombre camelcase se puede hacer, lo demas no es necesario
        PROVIDER.OPENAI: ("llm_tools.providers.openai_provider", "OpenAIProvider"),
        PROVIDER.MISTRAL: ("llm_tools.providers.mistral_provider", "MistralProvider"),
        PROVIDER.PHI: ("llm_tools.providers.phi_provider", "PhiProvider"),
        PROVIDER.QWEN: ("llm_tools.providers.qwen_provider", "QwenProvider"),
        PROVIDER.DEEPSEEK: ("llm_tools.providers.deepseek_provider", "DeepSeekProvider"),
        PROVIDER.GEMINI: ("llm_tools.providers.gemini_provider", "GeminiProvider"),
        PROVIDER.SBERT: ("llm_tools.providers.sbert_provider", "SBERTProvider"),
        PROVIDER.E5: ("llm_tools.providers.e5_provider", "E5Provider"),
        PROVIDER.BAAI: ("llm_tools.providers.baai_provider", "BAAIProvider"),
    }

    def __init__(self):
        super().__init__('llm')
        
        self.provider_map = {}

        self.active_llm = None
        self.active_embedding = None

        self.get_all_srv = self.create_service(GetModels, 'llm_tools/get_all_models', self.handle_get_all_models)
        self.get_active_srv = self.create_service(GetActiveModels, 'llm_tools/get_active_models', self.handle_get_active_models)
        self.get_available_srv = self.create_service(GetModels, 'llm_tools/get_available_models', self.handle_get_available_models)
        self.prompt_srv = self.create_service(Prompt, 'llm_tools/prompt', self.handle_prompt)
        self.embedding_srv = self.create_service(Embedding, 'llm_tools/embedding', self.handle_embedding)
        self.load_model_srv = self.create_service(LoadModel, 'llm_tools/load_model', self.handle_load_model)
        self.unload_model_srv = self.create_service(UnloadModel, 'llm_tools/unload_model', self.handle_unload_model)
        self.set_active_llm_srv = self.create_service(SetActiveModel, 'llm_tools/set_active_llm', self.handle_set_active_llm)
        self.set_active_embedding_srv = self.create_service(SetActiveModel, 'llm_tools/set_active_embedding', self.handle_set_active_embedding)

        self.get_logger().info("LLM Node initializated succesfully")

    def handle_get_all_models(self, request, response):
        provider_names = request.providers
        if not provider_names:
            provider_names = list(PROVIDER)

        self.get_logger().info(f"📖 Get All Models service for providers: {provider_names}")
        
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

    def handle_get_active_models(self, request, response):
        self.get_logger().info(f"📖 Get Active Models service.")

        if self.active_llm:
            [response.llm_provider, response.llm_model] = self.active_llm
        if self.active_embedding:
            [response.embedding_provider, response.embedding_model] = self.active_embedding
        
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
            provider_name, model_name = self._get_or_active("llm", request.provider, request.model)
            provider = self._get_provider(provider_name)

            result, model_used = provider.prompt(
                model=model_name,
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
            self._fill_response(response, False, str(e))
            self.get_logger().info(f"❌ Prompt service failed: {str(e)}")

        return response

    def handle_embedding(self, request, response):
        self.get_logger().info(f"📖 Embedding service for provider='{request.provider}', model='{request.model}'")
        
        try:
            provider_name, model_name = self._get_or_active("embedding", request.provider, request.model)
            provider = self._get_provider(provider_name)

            result, model_used = provider.embedding(
                model=model_name,
                user_input=request.user_input
            )

            response.embedding = result
            self._fill_response(response, True, "OK", provider_name, model_used)
            self.get_logger().info(f"✅ Embedding done using provider='{provider_name}', model='{model_used}'")
        except Exception as e:
            response.embedding = []
            self._fill_response(response, False, str(e))
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

    def handle_set_active_llm(self, request, response):
        response.success, response.message = self._set_active_model("llm", request.provider, request.model)

        return response

    def handle_set_active_embedding(self, request, response):
        response.success, response.message = self._set_active_model("embedding", request.provider, request.model)

        return response

    def _set_active_model(self, kind, provider, model):
        assert kind in ['llm', 'embedding']

        kind_upper = kind.upper()
        model_label = f"{kind_upper} model"

        self.get_logger().info(f"📖 Set Active Model service for {kind_upper}. Provider: {provider}, Model: {model}.")

        if not provider or not model:
            return False, "Provider and model must be specified"
        if provider not in list(PROVIDER):
            return False, f"Invalid {model_label.lower()} provider: {provider}"
        if model not in list(getattr(getattr(MODELS, kind_upper), provider.upper(), [])):
            return False, f"{model_label} '{model}' not found for provider '{provider}'"
        if provider not in self.provider_map:
            return False, f"Provider '{provider}' is not loaded. You must load it with your model first."
        if model not in self.provider_map[provider].get_active_models():
            return False, f"Provider '{provider}' is loaded but model '{model}' is not loaded. You must load it first."

        setattr(self, f"active_{kind}", [provider, model])
        
        return True, f"Active {model_label} set to {provider}/{model}"
    
    def _try_load_provider(self, name, api_key=""):
        if name not in self.provider_map:
            if name not in self.PROVIDER_CLASS_MAP:
                raise ValueError(f"Provider '{name}' is not supported.")
            
            module_path, class_name = self.PROVIDER_CLASS_MAP[name]
            try:
                module = importlib.import_module(module_path)
                provider_class = getattr(module, class_name)
            except Exception as e:
                raise ImportError(f"Could not load provider '{name}': {e}")
            
            self.provider_map[name] = provider_class(api_key=api_key) if api_key else provider_class()

        return self.provider_map[name]
    
    def _get_or_active(self, kind, provider_name, model_name):
        assert kind in ['llm', 'embedding']

        if not provider_name:
            active = getattr(self, f"active_{kind}", None)
            if active:
                return active
            else:
                raise ValueError(f"No provider specified and no active {kind.upper()} model")
        
        return provider_name, model_name

    def _get_provider(self, provider_name) -> BaseProvider:
        if provider_name in self.provider_map:
            return self.provider_map[provider_name]
        elif provider_name not in list(PROVIDER):
            raise ValueError(f"Provider {provider_name} does not exist.")
        else:
            raise ValueError(f"Provider {provider_name} is not loaded.")

    def _fill_response(self, response, success, message, provider="", model=""):
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