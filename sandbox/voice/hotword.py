import pvporcupine
import pyaudio
import struct

# Cambia esto por la ruta a tu archivo .ppn personalizado con la palabra "Sancho"
CUSTOM_WAKE_WORD_PATH = "sancho_linux.ppn"  # Asegúrate de que esta ruta es correcta

def main():
    # Inicializa Porcupine con el archivo de palabra personalizada
    porcupine = pvporcupine.create(keyword_paths=[CUSTOM_WAKE_WORD_PATH])

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

            result = porcupine.process(pcm)
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
