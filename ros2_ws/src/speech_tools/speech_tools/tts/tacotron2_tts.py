"""TODO: Add module documentation."""
from .coqui_single_tts import CoquiSingleTTS
from ..models import TTS_SPEAKERS


class Tacotron2TTS(CoquiSingleTTS):

"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        super().__init__(
            model="tts_models/es/mai/tacotron2-DDC"
        )

    def get_speakers(self) -> list[str]:
    """TODO: Describe get_speakers.
"""
        return list(TTS_SPEAKERS.TACOTRON2)
