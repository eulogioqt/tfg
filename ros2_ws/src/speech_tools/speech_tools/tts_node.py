import rclpy
import importlib
from rclpy.node import Node

from speech_msgs.msg import ModelSpeaker
from speech_msgs.srv import TTS, TTSModelSet, TTSModelGetAll, TTSModelGetActual

from .tts import TTSModel, TTS_MODELS, TTS_SPEAKERS

# hacer paquete audio_tools
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
        
        self.model: TTSModel = None # Poner parametro como en ros2web

        self.get_all_srv = self.create_service(TTSModelGetAll, 'speech_tools/tts/get_all_models', self.handle_get_all_models)
        self.get_actual_srv = self.create_service(TTSModelGetActual, 'speech_tools/tts/get_actual_model', self.handle_get_actual_model)
        self.tts_srv = self.create_service(TTS, 'speech_tools/tts', self.handle_tts)
        self.model_set_srv = self.create_service(TTSModelSet, 'speech_tools/tts/model_set', self.handle_model_set)

        self.get_logger().info('TTS Node inicializado correctamente')

    def handle_get_all_models(self, request, response):
        models_names = request.models
        if not models_names:
            models_names = list(TTS_MODELS)

        self.get_logger().info(f"📖 Get All Models service for models: {models_names}")
        
        response.speakers = []
        response.providers = []
        for model_name in models_names:
            if hasattr(TTS_MODELS, model_name.upper()):
                speakers = list(getattr(TTS_MODELS, model_name.upper()))
                response.speakers.append(ModelSpeaker(model=model_name, models=speakers))
                
            response.providers.append(model_name)

        return response

    def handle_get_actual_model(self, request, response):
        self.get_logger().info(f"📖 Get Actual Model service. Actual model: {self.model or 'No model'}")

        response.model = self.model or ""

        return response

    def handle_tts(self, request, response):
        text = request.text
        speaker = request.speaker

        try:
            audio, sample_rate = self.model.synthesize(text, speaker)

            response.audio = audio
            response.sample_rate = sample_rate
            self.get_logger().info(f"✅ TTS done using model: {self.model.__class__.__name__}")
        except Exception as e:
            self.get_logger().info(f"❌ TTS service failed: {str(e)}")

        response.audio = audio
        response.sample_rate = sample_rate

        return response

    def handle_model_set(self, request, response):
        self.get_logger().info(f"📖 Model set service for model {request.model}'")

        try:
            self._try_set_model(request.model)

            response.message = "OK"
            response.success = True
            self.get_logger().info(f"✅ Model {request.model} loaded succesfully")
        except Exception as e:
            response.message = str(e)
            response.success = False
            self.get_logger().info(f"❌ Set model service failed: {str(e)}")

        return response

    def _try_set_model(self, name):
        if self.model:
            pass # Borrar ese modelo, unload o algo, o el __del__ definirlo bien

        if name not in self.MODELS_CLASS_MAP:
            raise ValueError(f"Model '{name}' is not supported.")
        
        module_path, class_name = self.MODELS_CLASS_MAP[name]
        try:
            module = importlib.import_module(module_path)
            model_class = getattr(module, class_name)
        except Exception as e:
            raise ImportError(f"Could not set model '{name}': {e}")
        
        self.model = model_class()

def main(args=None):
    rclpy.init(args=args)
    
    tts = TTSNode()
    rclpy.spin(tts)
    
    rclpy.shutdown()