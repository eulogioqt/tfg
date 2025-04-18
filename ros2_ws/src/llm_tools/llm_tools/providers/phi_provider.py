from .hf_text_generation_provider import HFTextGenerationProvider

from ..prompt_formatters.phi_formatter import PhiFormatter
from ..models import MODELS


class PhiProvider(HFTextGenerationProvider):
    def __init__(self, models=None, api_key=None):
        super().__init__(
            models=models,
            api_key=api_key,
            model_formatters={
                MODELS.LLM.PHI.PHI_2: PhiFormatter
            }
        )
