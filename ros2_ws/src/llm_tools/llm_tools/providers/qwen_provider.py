"""TODO: Add module documentation."""
from .hf_emb_llm_provider import HFEmbLLMProvider
from ..models import PROVIDER


class QwenProvider(HFEmbLLMProvider):
"""TODO: Describe class."""
    def __init__(self, models=None):
    """TODO: Describe __init__.
Args:
    models (:obj:`Any`): TODO.
"""
        super().__init__(PROVIDER.QWEN, models)
