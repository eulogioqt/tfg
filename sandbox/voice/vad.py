import torch
import numpy as np
import pyaudio
import struct
import time
from scipy.signal import resample_poly

# Cargar modelo Silero VAD
model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', trust_repo=True)
(get_speech_ts, _, _, _, _) = utils

# Configuración de sample rates
INPUT_SAMPLE_RATE = 48000       # El que usa tu micro
TARGET_SAMPLE_RATE = 16000      # El que necesita Silero
FRAME_DURATION_MS = 30
FRAME_SIZE = int(INPUT_SAMPLE_RATE * FRAME_DURATION_MS / 1000)

def resample_audio(audio_np, orig_sr=48000, target_sr=16000):
    return resample_poly(audio_np, target_sr, orig_sr)

def main():
    pa = pyaudio.PyAudio()
    audio_stream = pa.open(
        rate=INPUT_SAMPLE_RATE,
        channels=1,
        format=pyaudio.paInt16,
        input=True,
        frames_per_buffer=FRAME_SIZE
    )

    print("🎤 Escuchando a 48kHz... (Ctrl+C para salir)")

    buffer = []

    try:
        while True:
            pcm = audio_stream.read(FRAME_SIZE, exception_on_overflow=False)
            pcm_int16 = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
            buffer.extend(pcm_int16)

            if len(buffer) >= INPUT_SAMPLE_RATE * 15:  # 15 segundos
                t_start = time.time()

                audio_np = np.array(buffer, dtype=np.float32)
                audio_resampled_np = resample_audio(audio_np)
                audio_resampled = torch.from_numpy(audio_resampled_np)

                t_resample = time.time() - t_start

                # VAD
                t_start = time.time()
                speech_timestamps = get_speech_ts(audio_resampled, model, sampling_rate=TARGET_SAMPLE_RATE)
                t_vad = time.time() - t_start

                if speech_timestamps:
                    print(f"🗣️ VOZ DETECTADA ({len(speech_timestamps)} segmentos) | Resample: {t_resample:.2f}s, VAD: {t_vad:.2f}s")
                else:
                    print(f"🔇 Sin voz | Resample: {t_resample:.2f}s, VAD: {t_vad:.2f}s")

                buffer = []

    except KeyboardInterrupt:
        print("\n🛑 Interrumpido por el usuario")

    finally:
        audio_stream.stop_stream()
        audio_stream.close()
        pa.terminate()

if __name__ == "__main__":
    main()
