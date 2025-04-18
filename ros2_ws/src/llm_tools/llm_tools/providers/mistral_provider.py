from .hf_text_generation_provider import HFTextGenerationProvider
from ..models import MODELS


class MistralProvider(HFTextGenerationProvider):
    def __init__(self, api_key):
        super().__init__([
            MODELS.LLM.MISTRAL.MISTRAL_7B, 
            MODELS.LLM.MISTRAL.MIXTRAL_8X7B
        ], api_key)