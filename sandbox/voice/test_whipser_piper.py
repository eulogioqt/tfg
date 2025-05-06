import time
import whisper
import torchaudio
import numpy as np
from piper import PiperVoice
from pyannote.audio import Inference
import sounddevice as sd
import torch
import soundfile as sf
import wave
from dotenv import load_dotenv
load_dotenv()
import os
HUGGING_FACE_API_KEY = os.getenv("HUGGING_FACE_API_KEY")

# ----- CONSTANTS -----
WHISPER_MODEL_SIZE = "large"
TARGET_SAMPLE_RATE = 16000
RECORD_SECONDS = 5

# Voice options
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

# These will be assigned dynamically
VOICE = None
PIPER_MODEL_PATH = None
PIPER_CONFIG_PATH = None

# ----- FUNCTIONS -----
def record_audio_in_memory(duration=RECORD_SECONDS, samplerate=TARGET_SAMPLE_RATE):
    print(f"🎙️ Recording {duration} seconds of audio at {samplerate} Hz...")
    audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype='float32')
    sd.wait()
    audio = audio.flatten()
    print(f"✅ Audio recorded in memory: shape = {audio.shape}")
    return audio, samplerate

def load_audio_file(filepath):
    print(f"📂 Loading audio from file: {filepath}")
    waveform, sample_rate = torchaudio.load(filepath)
    print(f"📈 Original sample rate: {sample_rate} Hz")

    # Convert to mono if stereo
    if waveform.shape[0] > 1:
        waveform = torch.mean(waveform, dim=0, keepdim=True)

    waveform = waveform.squeeze(0).numpy()

    if sample_rate != TARGET_SAMPLE_RATE:
        print(f"🔁 Resampling to {TARGET_SAMPLE_RATE} Hz...")
        waveform = torchaudio.functional.resample(torch.tensor(waveform), sample_rate, TARGET_SAMPLE_RATE).numpy()
        sample_rate = TARGET_SAMPLE_RATE

    print(f"✅ Loaded audio shape: {waveform.shape}")
    return waveform, sample_rate

def transcribe_audio_whisper_array(audio_array):
    print("🧠 Transcribing with Whisper...")
    start_time = time.time()
    model = whisper.load_model(WHISPER_MODEL_SIZE)
    result = model.transcribe(audio=audio_array, language="es", fp16=torch.cuda.is_available())
    duration = time.time() - start_time
    print(f"🕒 Transcription completed in {duration:.2f} seconds")
    print(f"📝 Transcribed text: {result['text']}")
    return result["text"], duration

def synthesize_and_play_with_piper(text):
    print("🗣️ Synthesizing and playing with Piper (stream_raw)...")
    
    # SÍNTESIS
    start_synthesis = time.time()
    voice = PiperVoice.load(PIPER_MODEL_PATH, config_path=PIPER_CONFIG_PATH)
    raw_audio_bytes = b"".join(voice.synthesize_stream_raw(text))
    audio_array = np.frombuffer(raw_audio_bytes, dtype=np.float32)
    synthesis_time = time.time() - start_synthesis
    print(f"🕒 Synthesis done in {synthesis_time:.2f} seconds")

    # GUARDAR CON WAVE COMO INT16
    filename = f"./audio_tts_{VOICES[VOICE]['name']}.wav"
    print(f"💾 Saving TTS audio to '{filename}'")
    with wave.open(filename, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2) 
        wf.setframerate(22050)
        wf.writeframes(audio_array.astype(np.float32).tobytes())
    print(f"💾 Saved TTS audio to './{filename}'")

    # REPRODUCCIÓN (si hay altavoces)
    print("🔊 Playing audio...")
    start_playback = time.time()
    sd.play(audio_array, samplerate=22050)
    sd.wait()
    playback_time = time.time() - start_playback
    print(f"🕒 Playback done in {playback_time:.2f} seconds")

    return synthesis_time, playback_time

def extract_speaker_embedding_from_array(audio_array):
    print("🧬 Extracting speaker embedding...")
    start_time = time.time()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    inference = Inference("pyannote/embedding", device=device, use_auth_token=HUGGING_FACE_API_KEY)

    # Pyannote expects shape (num_samples,)
    embedding = inference({
        "waveform": torch.tensor(audio_array).unsqueeze(0),
        "sample_rate": TARGET_SAMPLE_RATE
    })

    duration = time.time() - start_time
    print(f"🕒 Embedding extraction completed in {duration:.2f} seconds")
    print(f"📐 Embedding shape: {embedding.data.shape}")
    return embedding.data, duration

# ----- MAIN PIPELINE -----
def run_pipeline(audio_array):
    # STT
    text, stt_time = transcribe_audio_whisper_array(audio_array)

    # TTS
    tts_time, play_time = synthesize_and_play_with_piper(text)

    # Speaker Embedding
    embedding, embed_time = extract_speaker_embedding_from_array(audio_array)

    return {
        "text": text,
        "stt_time": stt_time,
        "tts_time": tts_time,
        "play_time": play_time,
        "embedding_time": embed_time,
        "embedding": embedding
    }

# ----- ENTRY POINT -----
if __name__ == "__main__":
    # 1. Select voice
    VOICE = input("Select voice (M for male / F for female): ").strip().upper()
    if VOICE not in VOICES:
        print("❌ Invalid choice. Use M or F.")
        exit(1)

    PIPER_MODEL_PATH = VOICES[VOICE]["model"]
    PIPER_CONFIG_PATH = VOICES[VOICE]["config"]
    print(f"🔈 Using voice: {VOICES[VOICE]['name']}")

    # 2. Optional audio path
    file_path = input("Enter path to audio file (or press Enter to record): ").strip()
    if file_path:
        if not os.path.isfile(file_path):
            print(f"❌ File not found: {file_path}")
            exit(1)
        audio_data, sr = load_audio_file(file_path)
    else:
        audio_data, sr = record_audio_in_memory()

    # Run pipeline
    result = run_pipeline(audio_data)

    # Final output
    print("\n✅ Final result:")
    print(f"Text: {result['text']}")
    print(f"STT time: {result['stt_time']:.2f} s")
    print(f"TTS synthesis time: {result['tts_time']:.2f} s")
    print(f"Playback time: {result['play_time']:.2f} s")
    print(f"Speaker embedding time: {result['embedding_time']:.2f} s")
    print(f"Embedding shape: {result['embedding'].shape}")
