"""TODO: Add module documentation."""
import re
import json

from abc import ABC, abstractmethod


class Prompt(ABC):

    @abstractmethod
"""TODO: Describe class."""
    def get_prompt_system(self):
    """TODO: Describe get_prompt_system.
"""
        pass

    @abstractmethod
    def get_user_prompt(self):
    """TODO: Describe get_user_prompt.
"""
        pass

    @abstractmethod
    def get_parameters(self):
    """TODO: Describe get_parameters.
"""
        pass


    @staticmethod
    def try_json_loads(text):
    """TODO: Describe try_json_loads.
Args:
    text (:obj:`Any`): TODO.
"""
        try:
            return json.loads(text)
        except Exception:
            return None

    @staticmethod
    def extract_json_from_code_block(text):
    """TODO: Describe extract_json_from_code_block.
Args:
    text (:obj:`Any`): TODO.
"""
        match = re.search(r"```json\s*(\{.*?\})\s*```", text, re.DOTALL)
        if match:
            return match.group(1).strip()
        
        match = re.search(r"(\{.*\})", text, re.DOTALL)
        if match:
            return match.group(1).strip()
        
        return None
