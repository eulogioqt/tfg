from ..constants import MODELS
from .hf_text_generation_provider import HFTextGenerationProvider


class LLaMAProvider(HFTextGenerationProvider):
    def __init__(self, api_key):
        super().__init__([MODELS.LLM.LLAMA.LLAMA3_8B], api_key)