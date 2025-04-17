import json

from .prompt_formatter import PromptFormatter

class GeminiFormatter(PromptFormatter):
    def format(self, prompt_system: str, messages_json: str, user_input: str) -> dict:
        history = []

        if messages_json:
            for msg in json.loads(messages_json):
                if msg["role"] == "user":
                    history.append({"role": "user", "parts": [msg["content"]]})
                elif msg["role"] == "assistant":
                    history.append({"role": "model", "parts": [msg["content"]]})

        if user_input:
            history.append({"role": "user", "parts": [user_input]})

        return {
            "system_instruction": prompt_system,
            "history": history
        }