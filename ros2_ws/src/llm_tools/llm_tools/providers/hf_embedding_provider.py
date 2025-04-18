import gc
import torch

from sentence_transformers import SentenceTransformer

from .base_provider import BaseProvider

# hacer a futuro que usando esto de hugging face se puedan poner todos los modelos
# algo tipo que el usuario lo defina en un yaml o un json o algo del proyecto
# ya vere como hacerlo eso es mas a futuro pero vonva

class HFEmbeddingProvider(BaseProvider):
    def __init__(self, models=None, api_key=None):
        self.api_key = api_key
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.models = {}
        
        if models:
            self.load(models)

    def embedding(self, user_input, model):
        model = model or list(self.models.keys())[0]
        if model not in self.models:
            raise ValueError(f"Model {model} not loaded in embedding provider.")
        
        return self.models[model].encode([user_input])[0].tolist()

    def prompt(self, *args, **kwargs):
        raise NotImplementedError("This provider does not support prompts.")

    def load(self, models):
        for model_enum in models:
            self.models[model_enum] = SentenceTransformer(model_enum, use_auth_token=self.api_key, device=self.device)

    def unload(self, models=None):
        if not models:
            models = list(self.models.keys())

        for model_enum in models:
            del self.models[model_enum]

        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
