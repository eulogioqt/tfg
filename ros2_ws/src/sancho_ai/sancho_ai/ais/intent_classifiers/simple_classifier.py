"""TODO: Add module documentation."""
import re

from .intent_classifier import IntentClassifier

from ...prompts.commands.commands import COMMANDS


class SimpleClassifier(IntentClassifier):
"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        self.provider_used = "SimpleNLP"
        self.model_used = "RegexRules"

    def classify(self, user_input: str, chat_history=[]):
    """TODO: Describe classify.
Args:
    user_input (:obj:`Any`): TODO.
    chat_history (:obj:`Any`): TODO.
"""
        user_input = user_input.lower().strip()

        if "foto" in user_input:
            return COMMANDS.TAKE_PICTURE, {}, self.provider_used, self.model_used

        match = re.match(r"^borra a (\w+)", user_input)
        if match:
            return COMMANDS.DELETE_USER, {"user": match.group(1)}, self.provider_used, self.model_used

        match = re.match(r"^renombra a (\w+) como (\w+)", user_input)
        if match:
            return COMMANDS.RENAME_USER, {
                "old_name": match.group(1),
                "new_name": match.group(2)
            }, self.provider_used, self.model_used

        return COMMANDS.UNKNOWN, {}, self.provider_used, self.model_used
