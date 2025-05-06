import numpy as np
import soundfile as sf
from bark import generate_audio, preload_models, SAMPLE_RATE

from .tts_model import TTSModel


class BarkTTS(TTSModel):

    def __init__(self, speaker="es_speaker_0"):
        self.speaker = speaker

        preload_models()

        self.sample_rate = SAMPLE_RATE
        self.speakers = ["es_speaker_0"]

    def synthesize(self, text: str) -> np.ndarray:
        audio = generate_audio(text, history_prompt=self.speaker)

        return audio
    
    def save(self, audio: np.ndarray, sample_rate: int, filename: str):
        if hasattr(audio, "numpy"):
            audio = audio.numpy()

        audio = np.clip(audio, -1.0, 1.0)
        sf.write(filename, audio, samplerate=sample_rate, subtype="PCM_16")