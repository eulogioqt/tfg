from .hf_embedding_provider import HFEmbeddingProvider
from llm_tools.constants import MODELS


class E5Provider(HFEmbeddingProvider):
    def __init__(self):
        super().__init__([MODELS.EMBEDDING.E5.E5_LARGE_V2])
