import time
import os
from datetime import datetime

from models import TTSModel, GoogleTTS, PiperTTS, CSS10TTS, XTTS, Tacotron2TTS, BarkTTS, YourTTS
import numpy as np
import sounddevice as sd

TTS_OPTIONS = {
    "bark": BarkTTS,
    "css10": CSS10TTS,
    "google": GoogleTTS,
    "piper": PiperTTS,
    "tacotron2": Tacotron2TTS,
    "xtts": XTTS,
    "your_tts": YourTTS
}
# repasar el get or default de los modelos y todo eos porque algunos esta la lista vacia y demas
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
        audio = tts_model.synthesize(text, speaker="")
        tts_time = time.time() - start

        times[tts_name] = tts_time

        speaker_suffix = f"_{tts_model.speaker}" if tts_model.speaker else ""
        filename = f"audio_tts_{tts_name}{speaker_suffix}.wav"
        filepath = os.path.join(output_dir, filename)
        
        print(type(audio))
        print(type(audio[100]))
        print(f"{audio[0]} {audio[234]}")
        print(np.mean(audio))
        print(np.max(audio))
        print(np.min(audio))
        tts_model.save(audio,filepath)
        sd.play(audio, samplerate=tts_model.get_sample_rate())
        sd.wait()

        print(f"✅ Guardado: {filepath}")

    for tts_name, tts_time in times.items():
        cpm = len(text) / tts_time

        print(f"\nTiempo de síntesis con {tts_name}: {tts_time:.2f}s (para sintetizar {len(text)} carácteres)")
        print(f"Carácteres por segundo con {tts_name}: {cpm:.2f}")