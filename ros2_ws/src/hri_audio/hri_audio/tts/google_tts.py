import wave
import numpy as np
from gtts import gTTS
from io import BytesIO
from pydub import AudioSegment

from .tts_model import TTSModel


class GoogleTTS(TTSModel):

    def synthesize(self, text: str, speaker: str) -> np.ndarray:
        if not speaker or speaker not in self.get_speakers():
            speaker = self.get_speakers()[0]
            
        mp3_fp = BytesIO()
        gTTS(text, lang="es", tld=speaker).write_to_fp(mp3_fp)
        mp3_fp.seek(0)

        audio = AudioSegment.from_mp3(mp3_fp).set_channels(1)
        audio = np.array(audio.get_array_of_samples()).astype(np.int16)

        return audio

    def save(self, audio: np.ndarray, filename: str):
        with wave.open(filename, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 2 bytes para int16
            wf.setframerate(self.get_sample_rate())
            wf.writeframes(audio.tobytes())

    def get_sample_rate(self) -> int:
        return 24000
    
    def get_speakers(self) -> list[str]:
        return ["es", "com.mx", "us"]