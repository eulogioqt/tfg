import torch
import numpy as np
import soundfile as sf
from bark import generate_audio, preload_models, SAMPLE_RATE

from .tts_model import TTSModel
from . import TTS_SPEAKERS


class BarkTTS(TTSModel):

    def __init__(self):
        use_gpu = torch.cuda.is_available()
        preload_models(
            text_use_gpu=use_gpu, 
            fine_use_gpu=use_gpu, 
            codec_use_gpu=use_gpu, 
            coarse_use_gpu=use_gpu
        )

    def synthesize(self, text: str, speaker: str) -> np.ndarray:
        if not speaker:
            speaker = self.get_speakers()[0]

        audio = generate_audio(text, history_prompt=speaker)

        return audio, speaker
    
    def save(self, audio: np.ndarray, filename: str):
        if hasattr(audio, "numpy"):
            audio = audio.numpy()

        audio = np.clip(audio, -1.0, 1.0)
        sf.write(filename, audio, samplerate=self.get_sample_rate(), subtype="PCM_16")

    def get_sample_rate(self) -> int:
        return SAMPLE_RATE
    
    def get_speakers(self) -> list[str]:
        return list(TTS_SPEAKERS.BARK)