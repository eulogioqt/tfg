import wave
import numpy as np
from gtts import gTTS
from io import BytesIO
from pydub import AudioSegment

from .tts_model import TTSModel


class GoogleTTS(TTSModel):
    def __init__(self):
        self.speaker = None
        
        self.sample_rate = 24000
        self.speakers = []

    def synthesize(self, text: str) -> np.ndarray:
        mp3_fp = BytesIO()
        gTTS(text, lang="es", tld="es").write_to_fp(mp3_fp)
        mp3_fp.seek(0)

        audio = AudioSegment.from_mp3(mp3_fp).set_channels(1)
        audio = np.array(audio.get_array_of_samples()).astype(np.int16)

        return audio

    def save(self, audio: np.ndarray, sample_rate: int, filename: str):
        with wave.open(filename, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 2 bytes para int16
            wf.setframerate(sample_rate)
            wf.writeframes(audio.tobytes())
