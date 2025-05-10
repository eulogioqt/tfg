from .interfaces import TTSInterface, HTTPException, JSONResponse


class TTSAPI(TTSInterface):

    def get_models(self):
        pass

    def load_model(self, model: str):
        pass

    def unload_model(self, model: str):
        pass

    def set_active_model(self, model: str, speaker: str):
        pass