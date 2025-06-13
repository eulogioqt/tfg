"""TODO: Add module documentation."""
from abc import ABC, abstractmethod


class IntentClassifier(ABC):

    @abstractmethod
"""TODO: Describe class."""
    def classify(self, user_input: str, chat_history: list = []):
    """TODO: Describe classify.
Args:
    user_input (:obj:`Any`): TODO.
    chat_history (:obj:`Any`): TODO.
"""
        pass
