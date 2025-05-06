import torch
import numpy as np
import soundfile as sf
from TTS.api import TTS

from .tts_model import TTSModel


class XTTS(TTSModel):

    def __init__(self, speaker="Alma María"):
        self.speaker = speaker

        self.model = TTS("tts_models/multilingual/multi-dataset/xtts_v2", progress_bar=False, gpu=torch.cuda.is_available())

        self.sample_rate = 24000
        self.speakers = list(self.model.synthesizer.tts_model.speaker_manager.name_to_id)

    def get_sample_rate(self) -> int:
        return self.sample_rate

    def get_speakers(self) -> list[str]:
        return self.speakers

    def synthesize(self, text: str) -> np.ndarray:
        audio = self.model.tts(text, speaker=self.speaker, speaker_wav=None, language="es")
        #np.frombuffer(audio_bytes, dtype=np.float32)

        return audio
    
    def save(self, audio: np.ndarray, sample_rate: int, filename: str):
        audio = np.clip(audio, -1.0, 1.0)
        sf.write(filename, audio, samplerate=sample_rate, subtype="PCM_16")