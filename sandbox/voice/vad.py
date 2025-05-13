import torch
import torchaudio
import numpy as np
import pyaudio
import struct
import time

# Cargar modelo Silero VAD
model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', trust_repo=True)
(get_speech_ts, _, _, _, _) = utils

# Configuración de sample rates
INPUT_SAMPLE_RATE = 48000       # El que usa tu micro o sistema
TARGET_SAMPLE_RATE = 16000      # El que necesita Silero
FRAME_DURATION_MS = 30
FRAME_SIZE = int(INPUT_SAMPLE_RATE * FRAME_DURATION_MS / 1000)

# Resampler (una sola vez)
resampler = torchaudio.transforms.Resample(orig_freq=INPUT_SAMPLE_RATE, new_freq=TARGET_SAMPLE_RATE)

def main():
    # Configura el micrófono
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
            pcm_int16 = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0  # Normaliza a [-1, 1]
            buffer.extend(pcm_int16)

            if len(buffer) >= INPUT_SAMPLE_RATE:  # 1 segundo a 48kHz
                t_start = time.time()

                # Tensor original (1 canal)
                audio_tensor = torch.tensor(buffer).unsqueeze(0)  # (1, N)

                # Resample a 16kHz
                audio_resampled = resampler(audio_tensor).squeeze(0)  # (N,)

                # VAD
                speech_timestamps = get_speech_ts(audio_resampled, model, sampling_rate=TARGET_SAMPLE_RATE)
                t_end = time.time()

                if speech_timestamps:
                    print(f"🗣️ VOZ DETECTADA ({len(speech_timestamps)} segmentos) | Proceso: {(t_end - t_start):.2f}s")
                else:
                    print(f"🔇 Sin voz | Proceso: {(t_end - t_start):.2f}s")

                buffer = []

    except KeyboardInterrupt:
        print("\n🛑 Interrumpido por el usuario")

    finally:
        audio_stream.stop_stream()
        audio_stream.close()
        pa.terminate()

if __name__ == "__main__":
    main()
