import torch
import numpy as np
from TTS.api import TTS

from .coqui_tts import CoquiTTS


class CoquiMultiTTS(CoquiTTS):

    def __init__(self, model, language):
        self.language = language

        self.model = TTS(model, progress_bar=False, gpu=torch.cuda.is_available())

    def synthesize(self, text: str, speaker: str) -> np.ndarray:
        if not speaker or speaker not in self.get_speakers():
            speaker = self.get_speakers()[0]
            
        audio = self.model.tts(text, speaker=speaker, speaker_wav=None, language=self.language)
        audio = np.array(audio)

        return audio

    def get_sample_rate(self) -> int:
        return self.model.synthesizer.output_sample_rate