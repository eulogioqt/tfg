from .hf_text_generation_provider import HFTextGenerationProvider
from .hf_embedding_provider import HFEmbeddingProvider
from .base_provider import BaseProvider

from ..models import MODELS, PROVIDER


class HFEmbdTextProvider(BaseProvider):
    def __init__(self, provider: PROVIDER, models=None, api_key=None, model_formatters=None):
        self.llm_models = getattr(MODELS.LLM, provider.upper())
        self.embedder_models = getattr(MODELS.EMBEDDING, provider.upper())

        self.llm = HFTextGenerationProvider(api_key=api_key, model_formatters=model_formatters)
        self.embedder = HFEmbeddingProvider(api_key=api_key)

        if models:
            self.load(models)

    def prompt(self, *args, **kwargs):
        return self.llm.prompt(*args, **kwargs)

    def embedding(self, *args, **kwargs):
        return self.embedder.embedding(*args, **kwargs)

    def load(self, models):
        for model_enum in models:
            if isinstance(model_enum, self.llm_models):
                self.llm.load([model_enum])
            elif isinstance(model_enum, self.embedder_models):
                self.embedder.load([model_enum])

    def unload(self, models=None):
        if not models:
            models = list(self.llm.models.keys()) + list(self.embedder.models.keys())

        for model_enum in models:
            if isinstance(model_enum, self.llm_models):
                self.llm.unload([model_enum])
            elif isinstance(model_enum, self.embedder_models):
                self.embedder.unload([model_enum])
                