"""TODO: Add module documentation."""
import json
from .prompt_formatter import PromptFormatter


class GeminiFormatter(PromptFormatter):
"""TODO: Describe class."""
    def format(self, prompt_system: str, messages_json: str, user_input: str) -> dict:
    """TODO: Describe format.
Args:
    prompt_system (:obj:`Any`): TODO.
    messages_json (:obj:`Any`): TODO.
    user_input (:obj:`Any`): TODO.
"""
        messages = []

        if prompt_system:
            messages.append({"role": "user", "parts": [prompt_system.strip()]})
            messages.append({"role": "model", "parts": ["Understood."]})

        if messages_json:
            for msg in json.loads(messages_json):
                if msg["role"] == "user":
                    messages.append({"role": "user", "parts": [msg["content"]]})
                elif msg["role"] == "assistant":
                    messages.append({"role": "model", "parts": [msg["content"]]})

        if user_input:
            messages.append({"role": "user", "parts": [user_input.strip()]})

        return messages
