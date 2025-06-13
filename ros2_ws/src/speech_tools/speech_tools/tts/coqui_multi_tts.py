"""TODO: Add module documentation."""
import torch
import numpy as np
from TTS.api import TTS

from .coqui_tts import CoquiTTS


class CoquiMultiTTS(CoquiTTS):

"""TODO: Describe class."""
    def __init__(self, model, language):
    """TODO: Describe __init__.
Args:
    model (:obj:`Any`): TODO.
    language (:obj:`Any`): TODO.
"""
        self.language = language

        self.model = TTS(model, progress_bar=False, gpu=torch.cuda.is_available())

    def synthesize(self, text: str, speaker: str) -> tuple[list[int], str]:
    """TODO: Describe synthesize.
Args:
    text (:obj:`Any`): TODO.
    speaker (:obj:`Any`): TODO.
"""
        if not speaker:
            speaker = self.get_speakers()[0]
            
        audio = self.model.tts(text, speaker=speaker, speaker_wav=None, language=self.language)
        audio = np.clip(audio, -1.0, 1.0)
        audio = (audio * 32767).astype(np.int16)

        max_val = np.max(np.abs(audio))
        if max_val > 0:
            scale = 32767 / max_val
            audio = (audio.astype(np.float32) * scale).clip(-32767, 32767).astype(np.int16)

        return audio.tolist(), speaker

    def get_sample_rate(self) -> int:
    """TODO: Describe get_sample_rate.
"""
        return self.model.synthesizer.output_sample_rate

    def unload(self):
    """TODO: Describe unload.
"""
        del self.model
        super().unload()
