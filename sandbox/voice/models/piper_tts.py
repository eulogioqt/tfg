import wave
import torch
import numpy as np
from piper import PiperVoice

from .tts_model import TTSModel


class PiperTTS(TTSModel):

    def __init__(self, speaker="davefx"):
        self.speaker = speaker

        use_cuda = torch.cuda.is_available()
        print(f"Using CUDA: {use_cuda}")
        self.model = PiperVoice.load(f"es_ES-{speaker}-medium.onnx", config_path=f"es_ES-{speaker}-medium.onnx.json", use_cuda=use_cuda)

        self.sample_rate = self.model.config.sample_rate
        self.speakers = ["davefx", "sharvard"]

    def synthesize(self, text: str) -> np.ndarray:
        stream = self.model.synthesize_stream_raw(text)
        audio_bytes = b"".join(stream)
        audio = np.frombuffer(audio_bytes, dtype=np.int16)

        return audio
    
    def save(self, audio: np.ndarray, sample_rate: int, filename: str):
        with wave.open(filename, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2) 
            wf.setframerate(sample_rate)
            wf.writeframes(audio.tobytes())