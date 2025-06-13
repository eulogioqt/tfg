"""TODO: Add module documentation."""
import json
from transformers import PreTrainedTokenizerBase
from .prompt_formatter import PromptFormatter


class HFChatTemplateFormatter(PromptFormatter):
"""TODO: Describe class."""
    def __init__(self, tokenizer: PreTrainedTokenizerBase):
    """TODO: Describe __init__.
Args:
    tokenizer (:obj:`Any`): TODO.
"""
        self.tokenizer = tokenizer

    def format(self, prompt_system: str, messages_json: str, user_input: str) -> str:
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
            
        return self.tokenizer.apply_chat_template(messages, add_generation_prompt=True, return_tensors="pt")
