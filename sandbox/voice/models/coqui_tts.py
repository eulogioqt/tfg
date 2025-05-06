import numpy as np
import soundfile as sf

from .tts_model import TTSModel


class CoquiTTS(TTSModel):

    def save(self, audio: np.ndarray, sample_rate: int, filename: str):
        audio = np.clip(audio, -1.0, 1.0)
        sf.write(filename, audio, samplerate=sample_rate, subtype="PCM_16")