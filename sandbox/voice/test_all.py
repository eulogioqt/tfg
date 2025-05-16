import time
import os
from datetime import datetime

from models import TTSModel, GoogleTTS, PiperTTS, CSS10TTS, XTTS, Tacotron2TTS, BarkTTS, YourTTS
import numpy as np

TTS_OPTIONS = {
    #"bark": BarkTTS,
    #"css10": CSS10TTS,
    #"google": GoogleTTS,
    #"piper": PiperTTS,
    #"tacotron2": Tacotron2TTS,
    "xtts": XTTS,
    #"your_tts": YourTTS,
}

# bark -> numpy float32
# css10 -> numpy float64
# google -> numpy int16
# piper -> numpy int16
# tacoctron2 -> numpy float64
# xtts -> numpy float64
# your_tts -> numpy float64

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
        audio, speaker_used = tts_model.synthesize(text, speaker="")
        tts_time = time.time() - start

        times[tts_name] = tts_time
        
        #save
        filename = f"audio_tts_{tts_name}.wav"
        filepath = os.path.join(output_dir, filename)
        TTSModel.save(filepath, audio, tts_model.get_sample_rate())
        
        # play
        TTSModel.play(audio, tts_model.get_sample_rate())

        print(f"✅ Guardado: {filepath}")

    for tts_name, tts_time in times.items():
        cpm = len(text) / tts_time

        print(f"\nTiempo de síntesis con {tts_name}: {tts_time:.2f}s (para sintetizar {len(text)} carácteres)")
        print(f"Carácteres por segundo con {tts_name}: {cpm:.2f}")