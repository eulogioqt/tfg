from .hf_embd_text_provider import HFEmbdTextProvider
from ..models import PROVIDER


class DeepSeekProvider(HFEmbdTextProvider):
    def __init__(self, models=None, api_key=None):
        super().__init__(PROVIDER.DEEPSEEK, models, api_key)