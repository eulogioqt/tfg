import json

from .prompt_formatter import PromptFormatter

class HFFormatter(PromptFormatter):
    def format(self, prompt_system: str, messages_json: str, user_input: str) -> str:
        full_prompt = ""

        if prompt_system:
            full_prompt += f"[INST] <<SYS>>\n{prompt_system}\n<</SYS>>\n"

        if messages_json:
            for msg in json.loads(messages_json):
                if msg["role"] == "user":
                    full_prompt += f"[INST] {msg['content']} [/INST]\n"
                elif msg["role"] == "assistant":
                    full_prompt += f"{msg['content']}\n"

        if user_input:
            full_prompt += f"\nInput: {user_input}"

        full_prompt += "\n[/INST]"
        return full_prompt