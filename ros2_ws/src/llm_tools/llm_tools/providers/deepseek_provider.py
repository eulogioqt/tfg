from .hf_text_generation_provider import HFTextGenerationProvider
from .hf_embedding_provider import HFEmbeddingProvider
from .base_provider import BaseProvider

from ..models import MODELS


class DeepSeekProvider(BaseProvider):
    def __init__(self, api_key):
        self.llm = HFTextGenerationProvider([MODELS.LLM.DEEPSEEK.DEEPSEEK_CHAT], api_key)
        self.embedder = HFEmbeddingProvider([MODELS.EMBEDDING.DEEPSEEK.DEEP_EMBED])

    def prompt(self, *args, **kwargs):
        return self.llm.prompt(*args, **kwargs)

    def embedding(self, *args, **kwargs):
        return self.embedder.embedding(*args, **kwargs)
