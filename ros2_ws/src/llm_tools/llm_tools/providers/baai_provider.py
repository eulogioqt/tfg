from .hf_embedding_provider import HFEmbeddingProvider
from ..models import MODELS


class BAAIProvider(HFEmbeddingProvider):
    def __init__(self):
        super().__init__([
            MODELS.EMBEDDING.BAAI.BGE_M3,
            MODELS.EMBEDDING.BAAI.BGE_BASE_EN
        ])
