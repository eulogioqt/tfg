import numpy as np
import torch
from scipy.signal import resample_poly
from collections import deque
from silero import get_speech_ts, model  # Suponiendo que ya has cargado Silero

# Configuración
INPUT_SR = 48000
TARGET_SR = 16000
CHUNK_SIZE = 1024
SILENCE_TIMEOUT = 0.8  # segundos de silencio para cortar
BUFFER_DURATION = 1.0  # ventana para análisis VAD

# Estados
WAITING = 0
RECORDING = 1

state = WAITING
voice_buffer = []
silence_duration = 0.0
vad_buffer = deque(maxlen=int(BUFFER_DURATION * INPUT_SR))  # para VAD
chunk_duration_sec = CHUNK_SIZE / INPUT_SR

def process_audio_chunk(chunk_bytes):
    global state, voice_buffer, silence_duration

    # Convertir a float32
    audio = np.frombuffer(chunk_bytes, dtype=np.int16).astype(np.float32) / 32768.0

    # Añadir a buffer circular de VAD
    vad_buffer.extend(audio)

    if len(vad_buffer) < INPUT_SR * BUFFER_DURATION:
        return None  # aún no hay suficiente para VAD

    # Resample buffer VAD
    vad_input = resample_poly(np.array(vad_buffer), TARGET_SR, INPUT_SR)
    vad_tensor = torch.from_numpy(vad_input).unsqueeze(0)

    # VAD
    speech = get_speech_ts(vad_tensor, model, sampling_rate=TARGET_SR)

    if speech:
        silence_duration = 0
        if state == WAITING:
            print("🟢 Comienza voz")
            state = RECORDING
            voice_buffer = []
        voice_buffer.append(chunk_bytes)

    else:
        if state == RECORDING:
            silence_duration += chunk_duration_sec
            voice_buffer.append(chunk_bytes)
            if silence_duration >= SILENCE_TIMEOUT:
                print("🔴 Fin de voz → enviar a STT")
                state = WAITING
                audio_bytes = b''.join(voice_buffer)
                voice_buffer = []
                return audio_bytes  # para pasar al STT

    return None
