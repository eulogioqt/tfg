"""TODO: Add module documentation."""
from .coqui_multi_tts import CoquiMultiTTS
from ..models import TTS_SPEAKERS


class XTTS(CoquiMultiTTS):

"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        super().__init__(
            model="tts_models/multilingual/multi-dataset/xtts_v2",
            language="es"    
        )

    def get_speakers(self) -> list[str]:
    """TODO: Describe get_speakers.
"""
        return list(TTS_SPEAKERS.XTTS)
