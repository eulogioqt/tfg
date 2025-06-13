"""TODO: Add module documentation."""
from .hf_embedding_provider import HFEmbeddingProvider


class BAAIProvider(HFEmbeddingProvider):
"""TODO: Describe class."""
    def __init__(self, models=None):
    """TODO: Describe __init__.
Args:
    models (:obj:`Any`): TODO.
"""
        super().__init__(models)
