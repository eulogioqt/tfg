from speech_msgs.srv import TTSGetActiveModel, TTSGetModels, TTSSetActiveModel, LoadModel, UnloadModel

from .service_engine import ServiceEngine


class TTSEngine(ServiceEngine):
    def __init__(self, node):
        super().__init__(node)

        self.get_models_cli = self.create_client(TTSGetModels, 'speech_tools/tts/get_all_models')
        self.get_active_cli = self.create_client(TTSGetActiveModel, 'speech_tools/tts/get_active_model')
        self.get_available_cli = self.create_client(TTSGetModels, 'speech_tools/tts/get_available_models')
        self.load_model_cli = self.create_client(LoadModel, 'speech_tools/tts/load_model')
        self.unload_model_cli = self.create_client(UnloadModel, 'speech_tools/tts/unload_model')
        self.set_active_cli = self.create_client(TTSSetActiveModel, 'speech_tools/tts/set_active_model')

        self.node.get_logger().info("TTS Engine initializated successfully")

    def get_all_models_request(self, models=[]):
        req = TTSGetModels.Request()
        req.models = models
        return self.call_service(self.get_models_cli, req)
    
    def get_active_model_request(self):
        req = TTSGetActiveModel.Request()
        return self.call_service(self.get_active_cli, req)

    def get_available_models_request(self, models=[]):
        req = TTSGetModels.Request()
        req.models = models
        return self.call_service(self.get_available_cli, req)

    def load_model_request(self, items):
        req = LoadModel.Request()
        req.items = items
        return self.call_service(self.load_model_cli, req)

    def unload_model_request(self, models):
        req = UnloadModel.Request()
        req.models = models
        return self.call_service(self.unload_model_cli, req)

    def set_active_model_request(self, model, speaker):
        req = TTSSetActiveModel.Request()
        req.model = model
        req.speaker = speaker
        return self.call_service(self.set_active_cli, req)