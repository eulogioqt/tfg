"""TODO: Add module documentation."""
import json

from .prompt_formatter import PromptFormatter


class OpenAIFormatter(PromptFormatter):
"""TODO: Describe class."""
    def format(self, prompt_system: str, messages_json: str, user_input: str) -> list:
    """TODO: Describe format.
Args:
    prompt_system (:obj:`Any`): TODO.
    messages_json (:obj:`Any`): TODO.
    user_input (:obj:`Any`): TODO.
"""
        messages = []

        if prompt_system:
            messages.append({"role": "system", "content": prompt_system})

        if messages_json:
            messages.extend(json.loads(messages_json))

        if user_input:
            messages.append({"role": "user", "content": user_input})

        return messages
