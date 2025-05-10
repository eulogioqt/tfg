from abc import ABC, abstractmethod

from .api_responses import APIResponse


class TTSInterface(ABC):

    @abstractmethod
    def get_models(self) -> APIResponse:
        pass

    @abstractmethod
    def load_model(self, model: str) -> APIResponse:
        pass

    @abstractmethod
    def unload_model(self, model: str) -> APIResponse:
        pass

    @abstractmethod
    def set_active_model(self, model: str, speaker: str) -> APIResponse:
        pass