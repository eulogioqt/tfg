"""TODO: Add module documentation."""
import os
import json
import random

from .response_generator import ResponseGenerator

from ...prompts.commands.commands import COMMANDS
from ...engines import HRIEngine


class TemplatesGenerator(ResponseGenerator):
"""TODO: Describe class."""
    def __init__(self, hri_engine: HRIEngine):
    """TODO: Describe __init__.
Args:
    hri_engine (:obj:`Any`): TODO.
"""
        self.hri_engine = hri_engine

        root_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        templates_path = os.path.join(root_dir, 'prompts', 'commands', 'templates.json')

        with open(templates_path, 'r', encoding='utf-8') as f:
            self.templates = json.load(f)

        self.provider_used = "Templates"
        self.model_used = "Template"

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
        templates_for_status = self.templates[intent][status]
        template = random.choice(templates_for_status)
        response = template.format(**arguments)
        emotion = "neutral"

        return response, emotion, self.provider_used, self.model_used

    def continue_conversation(self, user_input: str, chat_history: list) -> tuple[str, str, str]:
    """TODO: Describe continue_conversation.
Args:
    user_input (:obj:`Any`): TODO.
    chat_history (:obj:`Any`): TODO.
"""
        templates = self.templates[COMMANDS.UNKNOWN]["MISSING_ARGUMENT"]
        response = random.choice(templates)
        emotion = "neutral"

        return response, emotion, self.provider_used, self.model_used
