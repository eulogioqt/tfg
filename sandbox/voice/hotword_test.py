import os
import struct
import pvporcupine
import numpy as np
from scipy.signal import resample_poly
from dotenv import load_dotenv
import wave


class PVPorcupineHotword:
    def __init__(self):
        load_dotenv()

        device = os.environ.get("DEVICE")
        access_key = os.environ.get(f"PICOVOICE_API_KEY_{device}")
        wake_word_path = f"sancho_linux_{device.lower()}.ppn"
        model_path = "porcupine_params_es.pv"

        self.porcupine = pvporcupine.create(
            access_key=access_key,
            keyword_paths=[wake_word_path],
            model_path=model_path
        )

        self.buffer = np.array([], dtype=np.int16)

    def detect_hotword(self, audio_chunk: list[int], sample_rate: int) -> bool:
        audio_np = np.array(audio_chunk, dtype=np.int16)

        if sample_rate != self.porcupine.sample_rate:
            audio_np = audio_np.astype(np.float32) / 32768.0
            audio_np = resample_poly(audio_np, self.porcupine.sample_rate, sample_rate)
            audio_np = np.round(audio_np * 32768.0).astype(np.int16)

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


if __name__ == "__main__":
    hotword_detector = PVPorcupineHotword()

    wav_path = "sample.wav"

    with wave.open(wav_path, 'rb') as wf:
        sample_rate = wf.getframerate()
        num_channels = wf.getnchannels()
        sampwidth = wf.getsampwidth()

        if sampwidth != 2 or num_channels != 1:
            raise ValueError("⚠️ El archivo debe ser WAV mono con 16 bits por muestra.")

        frames = wf.readframes(wf.getnframes())
        audio_data = np.frombuffer(frames, dtype=np.int16).tolist()

    print(f"🎧 Procesando archivo {wav_path} a {sample_rate} Hz...")

    chunk_size = 1024  # Puedes ajustar esto si quieres más precisión o rendimiento

    for i in range(0, len(audio_data), chunk_size):
        chunk = audio_data[i:i + chunk_size]
        if hotword_detector.detect_hotword(chunk, sample_rate=sample_rate):
            print(f"🟢 Activación detectada en el chunk {i // chunk_size}")
