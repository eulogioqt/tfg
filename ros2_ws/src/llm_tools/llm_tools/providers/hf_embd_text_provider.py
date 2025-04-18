from .hf_text_generation_provider import HFTextGenerationProvider
from .hf_embedding_provider import HFEmbeddingProvider
from .base_provider import BaseProvider

from ..models import MODELS, PROVIDER


class HFEmbdTextProvider(BaseProvider):
    def __init__(self, provider: PROVIDER, models=None, api_key=None):
        self.llm_models = getattr(MODELS.LLM, provider.value.upper())
        self.embedder_models = getattr(MODELS.EMBEDDING, provider.value.upper())

        self.llm = HFTextGenerationProvider(api_key=api_key)
        self.embedder = HFEmbeddingProvider(api_key=api_key)

        if models:
            self.load(models)

    def prompt(self, *args, **kwargs):
        return self.llm.prompt(*args, **kwargs)

    def embedding(self, *args, **kwargs):
        return self.embedder.embedding(*args, **kwargs)

    def load(self, models):
        for model in models:
            if isinstance(model, self.llm_models):
                self.llm.load([model])
            elif isinstance(model, self.embedder_models):
                self.embedder.load([model])

    def unload(self, models):
        for model in models:
            if isinstance(model, self.llm_models):
                self.llm.unload([model])
            elif isinstance(model, self.embedder_models):
                self.embedder.unload([model])
