import torch
import numpy as np
from TTS.api import TTS

from .coqui_tts import CoquiTTS


class CoquiSingleTTS(CoquiTTS):

    def __init__(self, model):
        self.speaker = None
        
        self.model = TTS(model, progress_bar=False, gpu=torch.cuda.is_available())

        self.sample_rate = self.model.synthesizer.output_sample_rate
        self.speakers = []

    def synthesize(self, text: str) -> np.ndarray:
        audio = self.model.tts(text, speaker_wav=None)
        audio = np.array(audio)

        return audio