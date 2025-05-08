import os
import wave
import numpy as np
from piper import PiperVoice

from .tts_model import TTSModel
from . import TTS_SPEAKERS


class PiperTTS(TTSModel):

    def __init__(self):
        path_dave = os.path.join(os.path.dirname(__file__), "models/es_ES-davefx-medium.onnx")
        path_shar = os.path.join(os.path.dirname(__file__), "models/es_ES-sharvard-medium.onnx")

        self.models = {
            "davefx": PiperVoice.load(path_dave, config_path=f"{path_dave}.json"),
            "sharvard": PiperVoice.load(path_shar, config_path=f"{path_shar}.json"),
        } 

    def synthesize(self, text: str, speaker: str) -> np.ndarray:
        if not speaker:
            speaker = self.get_speakers()[0]

        stream = self.models[speaker].synthesize_stream_raw(text)
        audio_bytes = b"".join(stream)
        audio = np.frombuffer(audio_bytes, dtype=np.int16)

        return audio
    
    def save(self, audio: np.ndarray, filename: str):
        with wave.open(filename, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2) 
            wf.setframerate(self.get_sample_rate())
            wf.writeframes(audio.tobytes())

    def get_sample_rate(self) -> int:
        return next(iter(self.models.values())).config.sample_rate
    
    def get_speakers(self) -> list[str]:
        return list(TTS_SPEAKERS.PIPER)

    def unload(self):
        del self.models
        super().unload()