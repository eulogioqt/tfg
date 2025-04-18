from .hf_text_generation_provider import HFTextGenerationProvider

from ..prompt_formatters.phi_formatter import PhiFormatter
from ..models import MODELS


class PhiProvider(HFTextGenerationProvider):
    def __init__(self, api_key):
        super().__init__(
            model_enum_list=[MODELS.LLM.PHI.PHI_2],
            api_key=api_key,
            formatters={MODELS.LLM.PHI.PHI_2: PhiFormatter}
        )
