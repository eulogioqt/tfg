import torch
import numpy as np
from TTS.api import TTS

from .coqui_tts import CoquiTTS


class CoquiMultiTTS(CoquiTTS):

    def __init__(self, speaker, model, language):
        self.speaker = speaker
        self.language = language

        self.model = TTS(model, progress_bar=False, gpu=torch.cuda.is_available())

        self.sample_rate = self.model.synthesizer.output_sample_rate
        self.speakers = list(self.model.synthesizer.tts_model.speaker_manager.name_to_id)

    def synthesize(self, text: str) -> np.ndarray:
        audio = self.model.tts(text, speaker=self.speaker, speaker_wav=None, language=self.language)
        audio = np.array(audio)

        return audio