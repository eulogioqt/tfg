from .hf_embedding_provider import HFEmbeddingProvider
from ..models import MODELS


class SBERTProvider(HFEmbeddingProvider):
    def __init__(self):
        super().__init__([MODELS.EMBEDDING.SBERT.MINI_LM_L6_V2])
