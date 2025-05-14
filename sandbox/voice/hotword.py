import pvporcupine
import pyaudio
import struct
import time
import os

from dotenv import load_dotenv

load_dotenv()


# Cambia esto por la ruta a tu archivo .ppn personalizado con la palabra "Sancho"
CUSTOM_WAKE_WORD_PATH = "sancho_linux.ppn"  # Asegúrate de que esta ruta es correcta
PICOVOICE_API_KEY = os.environ.get("PICOVOICE_API_KEY")
MODEL_PATH = "porcupine_params_es.pv"

def main():
    # Inicializa Porcupine con el archivo de palabra personalizada
    porcupine = pvporcupine.create(
        access_key=PICOVOICE_API_KEY,  # Usa tu clave real aquí
        keyword_paths=[CUSTOM_WAKE_WORD_PATH],
        model_path=MODEL_PATH
    )


    # Configura el micrófono
    pa = pyaudio.PyAudio()
    audio_stream = pa.open(
        rate=porcupine.sample_rate,
        channels=1,
        format=pyaudio.paInt16,
        input=True,
        frames_per_buffer=porcupine.frame_length
    )

    print("🎤 Esperando la palabra clave 'Sancho'...")

    try:
        while True:
            pcm = audio_stream.read(porcupine.frame_length, exception_on_overflow=False)
            pcm = struct.unpack_from("h" * porcupine.frame_length, pcm)

            t = time.time()
            result = porcupine.process(pcm)
            print(f"Time to process {porcupine.frame_length} chunk: {(time.time() - t):.4f}")
            if result >= 0:
                print("✅ PALABRA DETECTADA: Sancho")

    except KeyboardInterrupt:
        print("\n🛑 Interrumpido por el usuario")

    finally:
        audio_stream.stop_stream()
        audio_stream.close()
        pa.terminate()
        porcupine.delete()

if __name__ == "__main__":
    main()
