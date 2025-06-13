"""TODO: Add module documentation."""
import gc
import torch

from sentence_transformers import SentenceTransformer

from .base_provider import BaseProvider


class HFEmbeddingProvider(BaseProvider):
"""TODO: Describe class."""
    def __init__(self, models=None, api_key=None):
    """TODO: Describe __init__.
Args:
    models (:obj:`Any`): TODO.
    api_key (:obj:`Any`): TODO.
"""
        self.api_key = api_key
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.models = {}
        
        if models:
            self.load(models)

    def embedding(self, user_input, model):
    """TODO: Describe embedding.
Args:
    user_input (:obj:`Any`): TODO.
    model (:obj:`Any`): TODO.
"""
        if not model:
            model = list(self.models.keys())[0]
        
        return self.models[model].encode([user_input])[0].tolist(), model

    def prompt(self, *args, **kwargs):
    """TODO: Describe prompt.
Args:
    *args (:obj:`Any`): TODO.
    **kwargs (:obj:`Any`): TODO.
"""
        raise NotImplementedError("This provider does not support prompts.")

    def load(self, models):
    """TODO: Describe load.
Args:
    models (:obj:`Any`): TODO.
"""
        for model_enum in models:
            self.models[model_enum] = SentenceTransformer(model_enum, use_auth_token=self.api_key, device=self.device)

    def unload(self, models=None):
    """TODO: Describe unload.
Args:
    models (:obj:`Any`): TODO.
"""
        if not models:
            models = self.get_active_models()

        for model_enum in models:
            del self.models[model_enum]

        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    def get_active_models(self):
    """TODO: Describe get_active_models.
"""
        return list(self.models.keys())
