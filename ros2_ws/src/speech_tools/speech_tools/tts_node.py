import gc
import ast
import rclpy
import importlib
from rclpy.node import Node

from speech_msgs.msg import ModelSpeaker, LoadUnloadResult
from speech_msgs.srv import TTS, TTSGetActiveModel, TTSGetModels, TTSSetActiveModel, LoadModel, UnloadModel

from .tts import TTSModel
from .models import TTS_MODELS, TTS_SPEAKERS


class TTSNode(Node):
    
    MODELS_CLASS_MAP = { # Poner esto mas cool, solo con el nombre camelcase se puede hacer, lo demas no es necesario
        TTS_MODELS.BARK: ("speech_tools.tts", "BarkTTS"),
        TTS_MODELS.CSS10: ("speech_tools.tts", "CSS10TTS"),
        TTS_MODELS.GOOGLE: ("speech_tools.tts", "GoogleTTS"),
        TTS_MODELS.PIPER: ("speech_tools.tts", "PiperTTS"),
        TTS_MODELS.TACOTRON2: ("speech_tools.tts", "Tacotron2TTS"),
        TTS_MODELS.XTTS: ("speech_tools.tts", "XTTS"),
        TTS_MODELS.YOUR_TTS: ("speech_tools.tts", "YourTTS"),
    }

    def __init__(self):
        super().__init__("tts")
        
        self.model_map = {}

        self.active_model = None

        self.get_all_srv = self.create_service(TTSGetModels, 'speech_tools/tts/get_all_models', self.handle_get_all_models)
        self.get_active_srv = self.create_service(TTSGetActiveModel, 'speech_tools/tts/get_active_model', self.handle_get_active_model)
        self.get_available_srv = self.create_service(TTSGetModels, 'speech_tools/tts/get_available_models', self.handle_get_available_models)
        self.tts_srv = self.create_service(TTS, 'speech_tools/tts', self.handle_tts)
        self.load_model_srv = self.create_service(LoadModel, 'speech_tools/tts/load_model', self.handle_load_model)
        self.unload_model_srv = self.create_service(UnloadModel, 'speech_tools/tts/unload_model', self.handle_unload_model)
        self.set_active_model_srv = self.create_service(TTSSetActiveModel, 'speech_tools/tts/set_active_model', self.handle_set_active_model)

        self._init_from_parameters()

        self.get_logger().info('TTS Node inicializado correctamente')

    def handle_get_all_models(self, request, response):
        models_names = set(request.models)
        if not models_names:
            models_names = list(TTS_MODELS)

        self.get_logger().info(f"📖 Get All Models service for models: {models_names}")
        
        response.speakers = []
        response.models = []
        for model_name in models_names:
            if hasattr(TTS_MODELS, model_name.upper()):
                speakers = list(getattr(TTS_MODELS, model_name.upper()))
                response.speakers.append(ModelSpeaker(model=model_name, speakers=speakers))
                if model_name not in response.models:
                    response.models.append(model_name)

        return response

    def handle_get_active_model(self, request, response):
        self.get_logger().info(f"📖 Get Active Model service.")

        if self.active_model:
            [response.model, response.speaker] = self.active_model

        return response

    def handle_get_available_models(self, request, response):
        models_names = set(request.models)
        if not models_names:
            models_names = list(TTS_MODELS)

        self.get_logger().info(f"📖 Get Available Models service for models: {models_names}")
        
        response.speakers = []
        response.models = []
        for model_name in models_names:
            if model_name in self.model_map:
                speakers = list(getattr(TTS_SPEAKERS, model_name.upper()))
                response.speakers.append(ModelSpeaker(model=model_name, speakers=speakers))

                response.models.append(model_name)

        return response

    def handle_tts(self, request, response):
        try:
            model_name, speaker_name = self._get_or_active(request.model, request.speaker)
            model = self._get_model(model_name)

            audio, speaker_used = model.synthesize(
                text=request.text, 
                speaker=speaker_name
            ) 

            self._fill_response(response, audio, model.get_sample_rate(), "OK", True, model_name, speaker_used)
            self.get_logger().info(f"✅ TTS done using model {model_name} and speaker {speaker_name}")
        except Exception as e:
            self._fill_response(response, [], 0, str(e), False)
            self.get_logger().info(f"❌ TTS service failed: {str(e)}")

        return response

    def handle_load_model(self, request, response):
        self.get_logger().info(f"📖 Load model service")

        response.results = []
        for item in request.items:
            result = LoadUnloadResult(model=item.model)
            try:
                model = self._try_load_model(item.model, item.api_key) # Con crearse ya esta cargado

                result.success, result.message = True, "Model loaded succesfully"
                self.get_logger().info(f"✅ Model {item.model} loaded succesfully")
            except Exception as e:
                result.success, result.message =  False, f"Error loading model: {str(e)}"
                self.get_logger().info(f"❌ Load model service failed for model {item.model}: {str(e)}")

            response.results.append(result)

        return response

    def handle_unload_model(self, request, response):
        self.get_logger().info(f"📖 Unload model service")
        
        response.results = []
        for model_name in request.models:
            result = LoadUnloadResult(model=model_name)

            if self.active_model and self.active_model[0] == model_name:
                result.success, result.message = False, f"Cannot unload model '{model_name}' because it is currently active"
                self.get_logger().warn(f"❌ Cannot unload active model '{model_name}'")
                response.results.append(result)
            elif model_name in self.model_map:
                try:
                    self.model_map[model_name].unload()
                    del self.model_map[model_name]
                    gc.collect()

                    result.success, result.message = True, "Models unloaded succesfully"
                    self.get_logger().info(f"✅ Model {model_name} unloaded succesfully")
                except Exception as e:
                    result.success, result.message = False, f"Error unloading model {model_name}: {str(e)}"
                    self.get_logger().info(f"❌ Unload model service failed for model {model_name}: {str(e)}")
            else:
                result.success, result.message = False, f"Model '{model_name}' not found."
                self.get_logger().info(f"❌ Load model service failed: Model '{model_name}' not found.")

            response.results.append(result)

        return response

    def handle_set_active_model(self, request, response):
        model = request.model
        speaker = request.speaker

        self.get_logger().info(f"📖 Set Active Model service. Model: {model}, Speaker: {speaker}.")

        if not model or not speaker:
            response.success, response.message = False, "Model and speaker must be specified"
        elif model not in list(TTS_MODELS):
            response.success, response.message = False, f"Invalid model: {model}"
        elif speaker not in list(getattr(TTS_SPEAKERS, model.upper(), [])):
            response.success, response.message = False, f"Speaker '{speaker}' not found for model '{model}'"
        elif model not in self.model_map:
            response.success, response.message = False, f"Model '{model}' is not loaded. You must load it first."
        else:
            self._set_active_model(model, speaker)
            response.success, response.message = True, f"Active model set to {model}/{speaker}"

        return response

    def _init_from_parameters(self):
        models_to_load = self.parse_string_list(self.declare_parameter("load_models", "[]").get_parameter_value().string_value)
        active_model = self.declare_parameter("active_model", "").get_parameter_value().string_value
        active_speaker = self.declare_parameter("active_speaker", "").get_parameter_value().string_value

        for [model_name, api_key] in models_to_load:
            try:
                self._try_load_model(model_name, api_key)
                self.get_logger().info(f"🔄 Model '{model_name}' loaded from parameter")
            except Exception as e:
                self.get_logger().error(f"❌ Could not load model '{model_name}' from parameter: {e}")

        if active_model and active_speaker:
            if active_model not in self.model_map:
                self.get_logger().warn(f"❌ Cannot set active model '{active_model}', it is not loaded")
            elif active_speaker not in list(getattr(TTS_SPEAKERS, active_model.upper())):
                self.get_logger().warn(f"❌ Cannot set active model. Speaker '{active_speaker}' not found")
            else:
                self._set_active_model(active_model, active_speaker)
                self.get_logger().info(f"✅ Active model set to '{active_model}/{active_speaker}' from parameter")

    def _set_active_model(self, model, speaker):
        self.active_model = [model, speaker]

    def _try_load_model(self, model_name, api_key=""):
        if model_name not in self.model_map:
            if model_name not in self.MODELS_CLASS_MAP:
                raise ValueError(f"Model '{model_name}' is not supported.")
            
            module_path, class_name = self.MODELS_CLASS_MAP[model_name]
            try:
                module = importlib.import_module(module_path)
                model_class = getattr(module, class_name)
            except Exception as e:
                raise ImportError(f"Could not load model '{model_name}': {e}")
            
            self.model_map[model_name] = model_class(api_key=api_key) if api_key else model_class()

        return self.model_map[model_name]

    def _get_or_active(self, model_name, speaker_name):
        if not model_name:
            if self.active_model:
                return self.active_model
            else:
                raise ValueError(f"No model specified and no active model")
        
        return model_name, speaker_name

    def _get_model(self, model_name) -> TTSModel:
        if model_name in self.model_map:
            return self.model_map[model_name]
        elif model_name not in list(TTS_MODELS):
            raise ValueError(f"Model {model_name} does not exist.")
        else:
            raise ValueError(f"Model {model_name} is not loaded.")

    def _fill_response(self, response, audio, sample_rate, message, success, model_used="", speaker_used=""):
        response.audio = audio
        response.sample_rate = sample_rate
        response.model_used = model_used
        response.speaker_used = speaker_used
        response.message = message
        response.success = success

    def parse_string_list(self, raw_string) -> list[str]:
        try:
            return ast.literal_eval(raw_string)
        except Exception:
            return []

def main(args=None):
    rclpy.init(args=args)
    
    tts = TTSNode()
    rclpy.spin(tts)
    
    rclpy.shutdown()