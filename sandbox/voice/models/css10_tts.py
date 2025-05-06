import torch
import numpy as np
import soundfile as sf
from TTS.api import TTS

from tts_model import TTSModel


class CSS10TTS(TTSModel):

    def __init__(self):
        self.model = TTS("tts_models/es/css10/vits", progress_bar=False, gpu=torch.cuda.is_available())

        self.sample_rate = self.model.synthesizer.output_sample_rate

    def get_sample_rate(self) -> int:
        return self.sample_rate

    def get_speakers(self) -> list[str]:
        return []

    def synthesize(self, text: str) -> np.ndarray:
        audio = self.model.tts(text, speaker_wav=None)
        audio = np.array(audio)

        return audio
    
    def save(self, audio: np.ndarray, sample_rate: int, filename: str):
        audio = np.clip(audio, -1.0, 1.0)
        sf.write(filename, audio, samplerate=sample_rate, subtype="PCM_16")