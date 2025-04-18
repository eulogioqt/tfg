from .hf_text_generation_provider import HFTextGenerationProvider


class MistralProvider(HFTextGenerationProvider):
    def __init__(self, models=None, api_key=None):
        super().__init__(models, api_key)