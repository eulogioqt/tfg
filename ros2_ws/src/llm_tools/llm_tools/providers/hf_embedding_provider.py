from sentence_transformers import SentenceTransformer

from .base_provider import BaseProvider


class HFEmbeddingProvider(BaseProvider):
    def __init__(self, model_enum_list: list):
        self.client = {}
        self.default_model = model_enum_list[0]

        for model_enum in model_enum_list:
            self.client[model_enum] = SentenceTransformer(model_enum.value)

    def embedding(self, model, user_input):
        model = model or self.default_model
        if model not in self.client:
            raise ValueError(f"Model {model} not loaded in embedding provider.")
        return self.client[model].encode([user_input])[0].tolist()

    def prompt(self, *args, **kwargs):
        raise NotImplementedError("This provider does not support prompts.")
