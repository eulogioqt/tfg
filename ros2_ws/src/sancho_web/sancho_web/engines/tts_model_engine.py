"""TODO: Add module documentation."""
from speech_msgs.msg import LoadModel as LoadModelMsg
from speech_msgs.srv import TTSGetActiveModel, TTSGetModels, TTSSetActiveModel, LoadModel, UnloadModel

from .service_engine import ServiceEngine


class TTSModelEngine(ServiceEngine):
"""TODO: Describe class."""
    def __init__(self, node):
    """TODO: Describe __init__.
Args:
    node (:obj:`Any`): TODO.
"""
        super().__init__(node)

        self.get_models_cli = self.create_client(TTSGetModels, 'speech_tools/tts/get_all_models')
        self.get_available_cli = self.create_client(TTSGetModels, 'speech_tools/tts/get_available_models')
        self.get_active_cli = self.create_client(TTSGetActiveModel, 'speech_tools/tts/get_active_model')
        self.load_model_cli = self.create_client(LoadModel, 'speech_tools/tts/load_model')
        self.unload_model_cli = self.create_client(UnloadModel, 'speech_tools/tts/unload_model')
        self.set_active_cli = self.create_client(TTSSetActiveModel, 'speech_tools/tts/set_active_model')

        self.node.get_logger().info("TTS Engine initializated successfully")

    def get_all_models_request(self, models=[]):
    """TODO: Describe get_all_models_request.
Args:
    models (:obj:`Any`): TODO.
"""
        req = TTSGetModels.Request()
        req.models = models
        
        result = self.call_service(self.get_models_cli, req)

        return [[i.model, i.needs_api_key, i.speakers] for i in result.speakers], result.models

    def get_available_models_request(self, models=[]):
    """TODO: Describe get_available_models_request.
Args:
    models (:obj:`Any`): TODO.
"""
        req = TTSGetModels.Request()
        req.models = models
        
        result = self.call_service(self.get_available_cli, req)

        return [[i.model, i.needs_api_key, i.speakers] for i in result.speakers], result.models

    def get_active_model_request(self):
    """TODO: Describe get_active_model_request.
"""
        req = TTSGetActiveModel.Request()

        result = self.call_service(self.get_active_cli, req)

        return result.model, result.speaker

    def load_model_request(self, items_list):
    """TODO: Describe load_model_request.
Args:
    items_list (:obj:`Any`): TODO.
"""
        req = LoadModel.Request()

        req.items = []
        for [model, api_key] in items_list:
            req.items.append(LoadModelMsg(model=model, api_key=api_key))
        
        result = self.call_service(self.load_model_cli, req)
        
        return [[i.model, i.message, i.success] for i in result.results]

    def unload_model_request(self, models):
    """TODO: Describe unload_model_request.
Args:
    models (:obj:`Any`): TODO.
"""
        req = UnloadModel.Request()
        req.models = models
        
        result = self.call_service(self.unload_model_cli, req)

        return [[i.model, i.message, i.success] for i in result.results]

    def set_active_model_request(self, model, speaker):
    """TODO: Describe set_active_model_request.
Args:
    model (:obj:`Any`): TODO.
    speaker (:obj:`Any`): TODO.
"""
        req = TTSSetActiveModel.Request()
        req.model = model
        req.speaker = speaker
        
        result = self.call_service(self.set_active_cli, req)

        return result.message, result.success
