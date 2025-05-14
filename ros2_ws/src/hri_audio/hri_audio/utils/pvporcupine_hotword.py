import os
import struct
import pvporcupine

import numpy as np

from scipy.signal import resample_poly
from dotenv import load_dotenv


class PVPorcupineHotwordDetector:
    def __init__(self):
        load_dotenv()

        access_key = os.environ.get("PICOVOICE_API_KEY")
        wake_word_path = os.path.join(os.path.dirname(__file__), "models/sancho_linux.ppn")
        model_path = os.path.join(os.path.dirname(__file__), "models/porcupine_params_es.pv")

        self.porcupine = pvporcupine.create(
            access_key=access_key,
            keyword_paths=[wake_word_path],
            model_path=model_path
        )

        self.buffer = np.array([], dtype=np.int16)
 
    def detect_hotword(self, audio_chunk: list[int], sample_rate: int) -> bool:
        audio_np = np.array(audio_chunk, dtype=np.int16)

        if sample_rate != self.porcupine.sample_rate:
            audio_np = resample_poly(audio_np, self.porcupine.sample_rate, sample_rate)
            print(type(audio_np[0]))
            print(audio_np[0])

        self.buffer = np.concatenate((self.buffer, audio_np))

        while len(self.buffer) >= self.porcupine.frame_length:
            frame = self.buffer[:self.porcupine.frame_length]
            self.buffer = self.buffer[self.porcupine.frame_length:]

            pcm = struct.pack("h" * self.porcupine.frame_length, *frame)
            pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)

            result = self.porcupine.process(pcm)
            if result >= 0:
                print("✅ HOTWORD DETECTADA: Sancho")
                return True

        return False
