from ..engines import LLMModelEngine
from .api_responses import HTTPException, JSONResponse, APIResponse


class LLMModelAPI:

    def __init__(self, node):
        self.engine = LLMModelEngine(node)

    def get_all_llm_models(self, providers: list[str] = []) -> APIResponse:
        models_response = []

        [all_provider_models, _] = self.engine.get_all_models_request(providers)
        [available_provider_models, _] = self.engine.get_available_models_request(providers)
        [active_provider, active_model] = self.engine.get_active_model_request()

        available_map = {
            provider: set(models) for [provider, _, models] in available_provider_models
        }

        for [provider, needs_api_key, models] in all_provider_models:
            for model in models:
                loaded = model in available_map.get(provider, set())
                active = provider == active_provider and model == active_model

                item = self._build_model_dict(provider, model, needs_api_key, loaded, active)
                models_response.append(item)

        return JSONResponse(content=models_response)

    def get_llm_model(self, provider: str, model: str) -> APIResponse:
        [all_provider_models, _] = self.engine.get_all_models_request([provider])
        if not all_provider_models:
            raise HTTPException(status_code=404, detail=f"Provider '{provider}' not found.")

        [available_provider_models, _] = self.engine.get_available_models_request([provider])
        [active_provider, active_model] = self.engine.get_active_model_request()

        models_all = next((m for m in all_provider_models if m[0] == provider), None)
        models_available = next((m for m in available_provider_models if m[0] == provider), None)

        if not models_all or model not in models_all[2]:
            raise HTTPException(status_code=404, detail=f"Model '{model}' not found for provider '{provider}'.")

        needs_api_key = models_all[1]
        loaded = model in (models_available[2] if models_available else [])
        active = provider == active_provider and model == active_model

        item = self._build_model_dict(provider, model, needs_api_key, loaded, active)

        return JSONResponse(content=item)

    def load_llm_model(self, provider: str, model: str, api_key: str) -> APIResponse:
        results = self.engine.load_model_request([[provider, [model], api_key]])
        [_, _, message, success] = results[0]

        return JSONResponse(content={
            "message": message,
            "success": success
        })

    def unload_llm_model(self, provider: str, model: str) -> APIResponse:
        results = self.engine.unload_model_request([[provider, [model]]])
        [_, _, message, success] = results[0]

        return JSONResponse(content={
            "message": message,
            "success": success
        })

    def set_active_llm_model(self, provider: str, model: str) -> APIResponse:
        message, success = self.engine.set_active_model_request(provider, model)

        return JSONResponse(content={
            "message": message,
            "success": success
        })

    def _build_model_dict(self, provider, model, needs_api_key, loaded, active):
        return {
            "provider": provider,
            "model": model,
            "needs_api_key": needs_api_key,
            "loaded": loaded,
            "active": active,
        }
