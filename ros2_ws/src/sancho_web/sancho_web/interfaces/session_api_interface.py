from abc import ABC, abstractmethod

from .api_responses import APIResponse


class SessionAPIInterface(ABC):

    @abstractmethod
    def get_all_sessions(self) -> APIResponse:
        pass

    @abstractmethod
    def get_session(self, faceprint_id: str) -> APIResponse:
        pass
