import wave
import numpy as np
from piper import PiperVoice

from .tts_model import TTSModel
from . import TTS_SPEAKERS


class PiperTTS(TTSModel):

    def __init__(self):
        self.models = {
            "davefx": PiperVoice.load(f"es_ES-davefx-medium.onnx", config_path=f"es_ES-davefx-medium.onnx.json"),
            "sharvard": PiperVoice.load(f"es_ES-sharvard-medium.onnx", config_path=f"es_ES-sharvard-medium.onnx.json"),
        } 

    def synthesize(self, text: str, speaker: str) -> np.ndarray:
        if not speaker or speaker not in self.get_speakers():
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
        return self.models[0].config.sample_rate
    
    def get_speakers(self) -> list[str]:
        return list(TTS_SPEAKERS.PIPER)