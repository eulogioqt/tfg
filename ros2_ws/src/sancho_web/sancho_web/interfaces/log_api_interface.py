from abc import ABC, abstractmethod

from .api_responses import APIResponse


class LogAPIInterface(ABC):

    @abstractmethod
    def get_all_logs(self, faceprint_id: str = None) -> APIResponse:
        pass

    @abstractmethod
    def get_log_by_id(self, id: str) -> APIResponse:
        pass