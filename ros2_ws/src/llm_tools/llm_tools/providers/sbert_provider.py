from .hf_embedding_provider import HFEmbeddingProvider


class SBERTProvider(HFEmbeddingProvider):
    def __init__(self, models=None, api_key=None):
        super().__init__(models, api_key)
