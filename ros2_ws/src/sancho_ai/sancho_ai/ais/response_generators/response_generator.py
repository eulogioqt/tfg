"""TODO: Add module documentation."""
from abc import ABC, abstractmethod


class ResponseGenerator(ABC):

    @abstractmethod
"""TODO: Describe class."""
    def generate_response(self, details: str, status: str, intent: str, 
                          arguments: dict, user_input: str, chat_history: list) -> str:
    """TODO: Describe generate_response.
Args:
    details (:obj:`Any`): TODO.
    status (:obj:`Any`): TODO.
    intent (:obj:`Any`): TODO.
    arguments (:obj:`Any`): TODO.
    user_input (:obj:`Any`): TODO.
    chat_history (:obj:`Any`): TODO.
"""
        pass

    @abstractmethod
    def continue_conversation(self, user_input: str, chat_history: list) -> str:
    """TODO: Describe continue_conversation.
Args:
    user_input (:obj:`Any`): TODO.
    chat_history (:obj:`Any`): TODO.
"""
        pass
