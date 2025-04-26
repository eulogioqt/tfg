from abc import ABC, abstractmethod

from .api_responses import APIResponse


class LogAPIInterface(ABC):

    @abstractmethod
    def get_all_logs(self) -> APIResponse:
        pass

    @abstractmethod
    def get_log(self, name: str) -> APIResponse:
        pass
