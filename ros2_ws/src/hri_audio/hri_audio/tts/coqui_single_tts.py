import torch
import numpy as np
from TTS.api import TTS

from .coqui_tts import CoquiTTS


class CoquiSingleTTS(CoquiTTS):

    def __init__(self, model):       
        self.model = TTS(model, progress_bar=False, gpu=torch.cuda.is_available())

    def synthesize(self, text: str, speaker: str) -> np.ndarray:
        audio = self.model.tts(text, speaker_wav=None)
        audio = np.array(audio)

        return audio

    def get_sample_rate(self) -> int:
        return self.model.synthesizer.output_sample_rate
    
    def get_speakers(self) -> list[str]:
        return []