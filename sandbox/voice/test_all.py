import time
import os
from datetime import datetime

from models import TTSModel, PiperTTS, CSS10TTS, XTTS, Tacotron2TTS, BarkTTS, YourTTS

TTS_OPTIONS = {
    "bark": BarkTTS,
    "css10": CSS10TTS,
    "piper": PiperTTS,
    "tacotron2": Tacotron2TTS,
    "xtts": XTTS,
    "your_tts": YourTTS
}

if __name__ == "__main__":
    text = input("📝 Introduce el texto a sintetizar: ").strip()
    if not text:
        print("❌ No se introdujo texto.")
        exit(1)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_dir = os.path.join("tests", timestamp)
    os.makedirs(output_dir, exist_ok=True)

    times = {}
    for tts_name, TTSClass in TTS_OPTIONS.items():
        print(f"\n🔊 Generando con {tts_name}...")

        tts_model: TTSModel = TTSClass()

        start = time.time()
        audio = tts_model.synthesize(text)
        tts_time = time.time() - start

        times[tts_name] = tts_time

        speaker_suffix = f"_{tts_model.speaker}" if tts_model.speaker else ""
        filename = f"audio_tts_{tts_name}{speaker_suffix}.wav"
        filepath = os.path.join(output_dir, filename)

        tts_model.save(audio, tts_model.get_sample_rate(), filepath)
        tts_model.play(audio, tts_model.get_sample_rate())

        print(f"✅ Guardado: {filepath}")

    for tts_name, tts_time in times.items():
        cpm = len(text) / tts_time

        print(f"\nTiempo de síntesis con {tts_name}: {tts_time:.2f}s (para sintetizar {len(text)} carácteres)")
        print(f"Carácteres por segundo con {tts_name}: {cpm:.2f}")