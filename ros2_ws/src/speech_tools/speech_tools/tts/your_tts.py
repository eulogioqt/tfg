"""TODO: Add module documentation."""
from .coqui_multi_tts import CoquiMultiTTS
from ..models import TTS_SPEAKERS


class YourTTS(CoquiMultiTTS):

"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        super().__init__(
            model="tts_models/multilingual/multi-dataset/your_tts",
            language="en"    
        )

    def get_speakers(self) -> list[str]:
    """TODO: Describe get_speakers.
"""
        return list(TTS_SPEAKERS.YOUR_TTS)
