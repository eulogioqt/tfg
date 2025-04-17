from ..constants import MODELS
from .hf_text_generation_provider import HFTextGenerationProvider

class PhiProvider(HFTextGenerationProvider):
    def __init__(self, api_key):
        super().__init__([MODELS.LLM.PHI.PHI_2], api_key)