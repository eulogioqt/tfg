"""TODO: Add module documentation."""
from .base import AI

from ..prompts.commands import COMMANDS


class DummyAI(AI):

"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        self.provider_used = "Dummy"
        self.model_used = "Dummy"

    def on_message(self, message, chat_history=[]):
    """TODO: Describe on_message.
Args:
    message (:obj:`Any`): TODO.
    chat_history (:obj:`Any`): TODO.
"""
        intent = COMMANDS.UNKNOWN
        arguments = {}

        value = {
            "text": message[::-1],
            "emotion": "neutral",
            "data": {}
        }

        return value, intent, arguments, self.provider_used, self.model_used
