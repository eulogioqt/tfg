import time
import os
import torch
import sounddevice as sd
import torchaudio
from faster_whisper import WhisperModel
from dotenv import load_dotenv
from pyannote.audio import Inference

from models import TTSModel, GoogleTTS, PiperTTS, CSS10TTS, XTTS, Tacotron2TTS, BarkTTS, YourTTS

load_dotenv()
HUGGING_FACE_API_KEY = os.getenv("HUGGING_FACE_API_KEY")

WHISPER_MODEL_SIZE = "large-v3"
TARGET_SAMPLE_RATE = 16000
RECORD_SECONDS = 5

VOICES = {
    "M": {
        "name": "davefx",
    },
    "F": {
        "name": "sharvard",
    }
}


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

faster_whisper_model = WhisperModel(WHISPER_MODEL_SIZE, device="cuda", compute_type="float16")

def transcribe_audio_whisper_array(audio_array):
    print("🧠 Transcribing with Faster-Whisper...")
    start = time.time()

    # Save audio temp (porque FasterWhisper usa ruta o archivo, no array directo)
    tmp_wav = "temp_input.wav"
    torchaudio.save(tmp_wav, torch.tensor(audio_array).unsqueeze(0), 16000)

    segments, info = faster_whisper_model.transcribe(tmp_wav, language="es")
    text = " ".join([segment.text for segment in segments])

    os.remove(tmp_wav)

    duration = time.time() - start
    print(f"📝 {text} ({duration:.2f}s)")
    return text, duration


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


def run_pipeline(audio_array, tts_model: TTSModel):
    text, stt_time = transcribe_audio_whisper_array(audio_array)

    print(f"🗣️ Synthesizing voice with {tts_model.__class__.__name__}...")
    start = time.time()
    audio = tts_model.synthesize(text)
    tts_time = time.time() - start

    duration = len(audio) / tts_model.get_sample_rate()

    filename = f"audio_tts_{tts_model.__class__.__name__.lower()}{f'_{tts_model.speaker}' if tts_model.speaker else ''}.wav"
    tts_model.save(audio, tts_model.get_sample_rate(), filename)
    tts_model.play(audio, tts_model.get_sample_rate(), wait=False)

    embedding, embed_time = extract_speaker_embedding_from_array(audio_array)

    return {
        "text": text,
        "stt_time": stt_time,
        "tts_time": tts_time,
        "play_time": duration,
        "embedding_time": embed_time,
        "embedding": embedding
    }


if __name__ == "__main__":
    tts_model: TTSModel = None

    TTS_OPTIONS = {
        "bark": BarkTTS,
        "css10": CSS10TTS,
        "google": GoogleTTS,
        "piper": PiperTTS,
        "tacotron2": Tacotron2TTS,
        "xtts": XTTS,
        "your_tts": YourTTS
    }

    TTS_ENGINE = input(f"TTS engine ({'/'.join(TTS_OPTIONS.keys())}): ").strip().lower()
    
    if TTS_ENGINE not in TTS_OPTIONS:
        print("❌ Invalid TTS engine.")
        exit(1)
    else:
        tts_model = TTS_OPTIONS[TTS_ENGINE]()

    file_path = "test.wav" # input("Enter path to audio file (or press Enter to record): ").strip()
    if file_path:
        if not os.path.isfile(file_path):
            print(f"❌ File not found: {file_path}")
            exit(1)
        audio_data, _ = load_audio_file(file_path)
    else:
        audio_data, _ = record_audio_in_memory()

    result = run_pipeline(audio_data, tts_model)
    total_time = result["stt_time"] + result["tts_time"] + result["embedding_time"]

    print("\n✅ Final result:")
    print(f"Text: {result['text']}")
    print(f"STT time: {result['stt_time']:.2f} s")
    print(f"TTS synthesis time: {result['tts_time']:.2f} s")
    print(f"Speaker embedding time: {result['embedding_time']:.2f} s")
    print(f"Embedding shape: {result['embedding'].shape}")
    print(f"\nTotal time: {total_time:.2f}s")
