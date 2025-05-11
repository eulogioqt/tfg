from ..engines import TTSModelEngine
from .api_responses import HTTPException, JSONResponse, APIResponse


class TTSModelAPI:

    def __init__(self, node):
        self.engine = TTSModelEngine(node)

    def get_models(self) -> APIResponse:
        pass

    def load_model(self, model: str) -> APIResponse:
        pass

    def unload_model(self, model: str) -> APIResponse:
        pass

    def set_active_model(self, model: str, speaker: str) -> APIResponse:
        pass