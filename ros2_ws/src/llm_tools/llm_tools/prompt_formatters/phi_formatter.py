"""TODO: Add module documentation."""
import json
from transformers import PreTrainedTokenizerBase
from .prompt_formatter import PromptFormatter


class PhiFormatter(PromptFormatter):
"""TODO: Describe class."""
    def __init__(self, tokenizer: PreTrainedTokenizerBase):
    """TODO: Describe __init__.
Args:
    tokenizer (:obj:`Any`): TODO.
"""
        self.tokenizer = tokenizer

    def format(self, prompt_system: str, messages_json: str, user_input: str):
    """TODO: Describe format.
Args:
    prompt_system (:obj:`Any`): TODO.
    messages_json (:obj:`Any`): TODO.
    user_input (:obj:`Any`): TODO.
"""
        instruction = prompt_system + "\n" if prompt_system else ""

        if messages_json:
            for msg in json.loads(messages_json):
                role = msg["role"].capitalize()
                content = msg["content"]
                instruction += f"{role}: {content}\n"

        if user_input:
            instruction += f"User: {user_input}\n"

        full_prompt = f"Instruct: {instruction.strip()}\nOutput:"

        return self.tokenizer(full_prompt, return_tensors="pt", return_attention_mask=False)["input_ids"]
