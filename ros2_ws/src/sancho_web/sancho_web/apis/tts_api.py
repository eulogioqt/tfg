from .api_responses import HTTPException, JSONResponse, APIResponse


class TTSAPI:

    def get_models(self) -> APIResponse:
        pass

    def load_model(self, model: str) -> APIResponse:
        pass

    def unload_model(self, model: str) -> APIResponse:
        pass

    def set_active_model(self, model: str, speaker: str) -> APIResponse:
        pass