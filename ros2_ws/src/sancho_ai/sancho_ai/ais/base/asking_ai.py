"""TODO: Add module documentation."""
from abc import ABC, abstractmethod

class AskingAI:

    @abstractmethod
"""TODO: Describe class."""
    def get_name(self, message: str):
    """TODO: Describe get_name.
Args:
    message (:obj:`Any`): TODO.
"""
        pass
    
    @abstractmethod
    def confirm_name(self, message: str):
    """TODO: Describe confirm_name.
Args:
    message (:obj:`Any`): TODO.
"""
        pass
