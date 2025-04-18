from .hf_text_generation_provider import HFTextGenerationProvider

from ..prompt_formatters.phi_formatter import PhiFormatter
from ..models import MODELS


class PhiProvider(HFTextGenerationProvider):
    def __init__(self, models=None):
        super().__init__(models=models, model_formatters={
                MODELS.LLM.PHI.PHI_2: PhiFormatter
            }
        )
