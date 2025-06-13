"""TODO: Add module documentation."""
from .hf_llm_provider import HFLLMProvider


class YIProvider(HFLLMProvider):
"""TODO: Describe class."""
    def __init__(self, models=None):
    """TODO: Describe __init__.
Args:
    models (:obj:`Any`): TODO.
"""
        super().__init__(models)
