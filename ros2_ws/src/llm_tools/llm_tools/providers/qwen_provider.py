from .hf_embd_text_provider import HFEmbdTextProvider
from ..models import PROVIDER


class QwenProvider(HFEmbdTextProvider):
    def __init__(self, models=None, api_key=None):
        super().__init__(PROVIDER.QWEN, models, api_key)