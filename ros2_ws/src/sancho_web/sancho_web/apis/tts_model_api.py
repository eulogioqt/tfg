from ..engines import TTSModelEngine
from .api_responses import HTTPException, JSONResponse, APIResponse


class TTSModelAPI:

    def __init__(self, node):
        self.engine = TTSModelEngine(node)

    def get_all_tts_models(self, models: list[str] = []) -> APIResponse:
        pass

    def get_tts_model(self, model: str) -> APIResponse:
        pass

    def load_tts_model(self, model: str, api_key: str) -> APIResponse:
        pass

    def unload_tts_model(self, model: str) -> APIResponse:
        pass

    def set_active_tts_model(self, model: str, speaker: str) -> APIResponse:
        pass