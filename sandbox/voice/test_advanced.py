import time
import os
import wave
import torch
import numpy as np
import whisper
import torchaudio
import sounddevice as sd
from piper import PiperVoice
from pyannote.audio import Inference
from dotenv import load_dotenv
from TTS.api import TTS  # For XTTS
from bark import generate_audio, preload_models  # For Bark
from scipy.io.wavfile import write as write_wav
import random

load_dotenv()
HUGGING_FACE_API_KEY = os.getenv("HUGGING_FACE_API_KEY")

WHISPER_MODEL_SIZE = "medium"
TARGET_SAMPLE_RATE = 16000
RECORD_SECONDS = 5
TTS_ENGINE = None

VOICES = {
    "M": {
        "name": "davefx",
        "model": "es_ES-davefx-medium.onnx",
        "config": "es_ES-davefx-medium.onnx.json"
    },
    "F": {
        "name": "sharvard",
        "model": "es_ES-sharvard-medium.onnx",
        "config": "es_ES-sharvard-medium.onnx.json"
    }
}

VOICE = None
PIPER_MODEL_PATH = None
PIPER_CONFIG_PATH = None


def record_audio_in_memory(duration=RECORD_SECONDS, samplerate=TARGET_SAMPLE_RATE):
    print(f"🎙️ Recording {duration} seconds...")
    audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype='float32')
    sd.wait()
    return audio.flatten(), samplerate

def load_audio_file(filepath):
    waveform, sample_rate = torchaudio.load(filepath)
    if waveform.shape[0] > 1:
        waveform = torch.mean(waveform, dim=0, keepdim=True)
    waveform = waveform.squeeze(0).numpy()
    if sample_rate != TARGET_SAMPLE_RATE:
        waveform = torchaudio.functional.resample(torch.tensor(waveform), sample_rate, TARGET_SAMPLE_RATE).numpy()
    return waveform, TARGET_SAMPLE_RATE

def transcribe_audio_whisper_array(audio_array):
    print("🧠 Transcribing with Whisper...")
    start = time.time()
    model = whisper.load_model(WHISPER_MODEL_SIZE)
    result = model.transcribe(audio=audio_array, language="es", fp16=torch.cuda.is_available())
    duration = time.time() - start
    print(f"📝 {result['text']} ({duration:.2f}s)")
    return result["text"], duration

def synthesize_and_play_with_piper(text):
    print("🗣️ TTS with Piper...")
    voice = PiperVoice.load(PIPER_MODEL_PATH, config_path=PIPER_CONFIG_PATH)

    start = time.time()
    stream = voice.synthesize_stream_raw(text)
    audio_bytes = b"".join(stream)
    audio = np.frombuffer(audio_bytes, dtype=np.float32)
    tts_time = time.time() - start

    save_and_play_piper(audio, 22050, f"piper_{VOICES[VOICE]['name']}")
    return tts_time, audio.shape[0] / 22050

def synthesize_and_play_with_xtts(text):
    print("🗣️ TTS with XTTS...")
    model = TTS("tts_models/multilingual/multi-dataset/xtts_v2", progress_bar=False, gpu=torch.cuda.is_available())
    speaker = random.choice(list(model.synthesizer.tts_model.speaker_manager.name_to_id))
    speaker = "Alma María"

    start = time.time()
    audio = model.tts(text, speaker=speaker, speaker_wav=None, language="es")
    tts_time = time.time() - start
    
    save_and_play_xtts(audio, 24000, f"xtts_{speaker.replace(' ', '_')}")
    return tts_time, len(audio) / 24000

def synthesize_and_play_with_bark(text):
    print("🗣️ TTS with Bark...")
    preload_models()

    start = time.time()
    audio = generate_audio(text, history_prompt="es_speaker_0")
    tts_time = time.time() - start
    
    save_and_play_bark(audio, 22050, "bark")
    return tts_time, audio.shape[0] / 22050

def save_and_play_piper(audio, sample_rate, engine_name):
    filename = f"audio_tts_{engine_name}.wav"
    print(f"💾 Saving TTS audio to '{filename}'")
    with wave.open(filename, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2) 
        wf.setframerate(sample_rate)
        wf.writeframes(audio.astype(np.float32).tobytes())
    print(f"💾 Saved TTS audio to './{filename}'")
    try:
        sd.play(audio, samplerate=sample_rate)
        #sd.wait()
    except Exception:
        print("🔇 Playback skipped (no audio output)")

def save_and_play_xtts(audio, sample_rate, engine_name):
    import soundfile as sf
    filename = f"audio_tts_{engine_name}.wav"
    print(f"💾 Saving TTS audio to '{filename}'")
    
    # Clip por seguridad
    audio = np.clip(audio, -1.0, 1.0)
    sf.write(filename, audio, samplerate=sample_rate, subtype="PCM_16")
    print(f"💾 Saved TTS audio to './{filename}'")

    try:
        sd.play(audio, samplerate=sample_rate)
        #sd.wait()
    except Exception:
        print("🔇 Playback skipped (no audio output)")

def save_and_play_bark(audio, sample_rate, engine_name):
    import soundfile as sf
    filename = f"audio_tts_{engine_name}.wav"
    print(f"💾 Saving TTS audio to '{filename}'")

    # Convertir de torch.Tensor a numpy si es necesario
    if hasattr(audio, "numpy"):
        audio = audio.numpy()

    audio = np.clip(audio, -1.0, 1.0)
    sf.write(filename, audio, samplerate=sample_rate, subtype="PCM_16")
    print(f"💾 Saved TTS audio to './{filename}'")

    try:
        sd.play(audio, samplerate=sample_rate)
        #sd.wait()
    except Exception:
        print("🔇 Playback skipped (no audio output)")

def extract_speaker_embedding_from_array(audio_array):
    print("🧬 Extracting speaker embedding...")
    start = time.time()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    inference = Inference("pyannote/embedding", device=device, use_auth_token=HUGGING_FACE_API_KEY)
    embedding = inference({
        "waveform": torch.tensor(audio_array).unsqueeze(0),
        "sample_rate": TARGET_SAMPLE_RATE
    })
    duration = time.time() - start
    print(f"📐 Embedding shape: {embedding.data.shape} ({duration:.2f}s)")
    return embedding.data, duration

def run_pipeline(audio_array):
    text, stt_time = transcribe_audio_whisper_array(audio_array)

    if TTS_ENGINE == "piper":
        tts_time, play_secs = synthesize_and_play_with_piper(text)
    elif TTS_ENGINE == "xtts":
        tts_time, play_secs = synthesize_and_play_with_xtts(text)
    elif TTS_ENGINE == "bark":
        tts_time, play_secs = synthesize_and_play_with_bark(text)
    else:
        raise ValueError("Invalid TTS engine")

    embedding, embed_time = extract_speaker_embedding_from_array(audio_array)

    return {
        "text": text,
        "stt_time": stt_time,
        "tts_time": tts_time,
        "play_time": play_secs,
        "embedding_time": embed_time,
        "embedding": embedding
    }

if __name__ == "__main__":
    TTS_ENGINE = input("TTS engine (piper/xtts/bark): ").strip().lower()
    if TTS_ENGINE not in {"piper", "xtts", "bark"}:
        print("❌ Invalid TTS engine.")
        exit(1)

    if TTS_ENGINE == "piper":
        VOICE = input("Select voice (M/F): ").strip().upper()
        if VOICE not in VOICES:
            print("❌ Invalid voice.")
            exit(1)
        PIPER_MODEL_PATH = VOICES[VOICE]["model"]
        PIPER_CONFIG_PATH = VOICES[VOICE]["config"]

    file_path = input("Enter path to audio file (or press Enter to record): ").strip()
    if file_path:
        if not os.path.isfile(file_path):
            print(f"❌ File not found: {file_path}")
            exit(1)
        audio_data, _ = load_audio_file(file_path)
    else:
        audio_data, _ = record_audio_in_memory()

    result = run_pipeline(audio_data)

    print("\n✅ Final result:")
    print(f"Text: {result['text']}")
    print(f"STT time: {result['stt_time']:.2f} s")
    print(f"TTS synthesis time: {result['tts_time']:.2f} s")
    print(f"Playback duration: {result['play_time']:.2f} s")
    print(f"Speaker embedding time: {result['embedding_time']:.2f} s")
    print(f"Embedding shape: {result['embedding'].shape}")
