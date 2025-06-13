"""TODO: Add module documentation."""
from abc import ABC, abstractmethod

class PromptFormatter(ABC):
    @abstractmethod
"""TODO: Describe class."""
    def format(self, prompt_system: str, messages_json: str, user_input: str) -> str | dict:
    """TODO: Describe format.
Args:
    prompt_system (:obj:`Any`): TODO.
    messages_json (:obj:`Any`): TODO.
    user_input (:obj:`Any`): TODO.
"""
        pass
