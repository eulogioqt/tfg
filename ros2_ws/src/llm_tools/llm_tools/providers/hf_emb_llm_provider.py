"""TODO: Add module documentation."""
from .hf_llm_provider import HFLLMProvider
from .hf_embedding_provider import HFEmbeddingProvider
from .base_provider import BaseProvider

from ..models import MODELS, PROVIDER


class HFEmbLLMProvider(BaseProvider):
"""TODO: Describe class."""
    def __init__(self, provider: PROVIDER, models=None, api_key=None, model_formatters=None):
    """TODO: Describe __init__.
Args:
    provider (:obj:`Any`): TODO.
    models (:obj:`Any`): TODO.
    api_key (:obj:`Any`): TODO.
    model_formatters (:obj:`Any`): TODO.
"""
        self.llm_models = list(getattr(MODELS.LLM, provider.upper()))
        self.embedder_models = list(getattr(MODELS.EMBEDDING, provider.upper()))
        
        self.llm = HFLLMProvider(api_key=api_key, model_formatters=model_formatters)
        self.embedder = HFEmbeddingProvider(api_key=api_key)

        if models:
            self.load(models)

    def prompt(self, *args, **kwargs):
    """TODO: Describe prompt.
Args:
    *args (:obj:`Any`): TODO.
    **kwargs (:obj:`Any`): TODO.
"""
        return self.llm.prompt(*args, **kwargs)

    def embedding(self, *args, **kwargs):
    """TODO: Describe embedding.
Args:
    *args (:obj:`Any`): TODO.
    **kwargs (:obj:`Any`): TODO.
"""
        return self.embedder.embedding(*args, **kwargs)

    def load(self, models):
    """TODO: Describe load.
Args:
    models (:obj:`Any`): TODO.
"""
        for model_enum in models:
            if model_enum in self.llm_models:
                self.llm.load([model_enum])
            elif model_enum in self.embedder_models:
                self.embedder.load([model_enum])

    def unload(self, models=None):
    """TODO: Describe unload.
Args:
    models (:obj:`Any`): TODO.
"""
        if not models:
            models = self.get_active_models()

        for model_enum in models:
            if model_enum in self.llm_models:
                self.llm.unload([model_enum])
            elif model_enum in self.embedder_models:
                self.embedder.unload([model_enum])

    def get_active_models(self):
    """TODO: Describe get_active_models.
"""
        return list(self.llm.models.keys()) + list(self.embedder.models.keys())
