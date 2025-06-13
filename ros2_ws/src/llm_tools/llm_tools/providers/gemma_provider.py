"""TODO: Add module documentation."""
from .hf_llm_provider import HFLLMProvider


class GemmaProvider(HFLLMProvider):
"""TODO: Describe class."""
    def __init__(self, models=None, api_key=None):
    """TODO: Describe __init__.
Args:
    models (:obj:`Any`): TODO.
    api_key (:obj:`Any`): TODO.
"""
        super().__init__(models, api_key)
