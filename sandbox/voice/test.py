import os
from TTS.api import TTS

# Inicializar el modelo XTTS
tts = TTS(model_name="tts_models/multilingual/multi-dataset/xtts_v2", progress_bar=False, gpu=True)

# Lista de voces disponibles (puedes filtrarlas manualmente si sabes cuáles son masculinas)
all_speakers = list(tts.speakers.keys())

# Aquí defines una lista manual de las voces que te suenan masculinas
masculine_voices = [
    'Andrew Chipper', 'Badr Odhiambo', 'Dionisio Schuyler', 'Royston Min', 'Viktor Eka',
    'Abrahan Mack', 'Adde Michal', 'Baldur Sanjin', 'Craig Gutsy', 'Damien Black',
    'Gilberto Mathias', 'Ilkin Urbano', 'Kazuhiko Atallah', 'Ludvig Milivoj', 'Suad Qasim',
    'Torcull Diarmuid', 'Viktor Menelaos', 'Zacharie Aimilios', 'Ferran Simen',
    'Xavier Hayasaka', 'Luis Moray', 'Marcos Rudaski', 'Eugenio Mataracı',
    'Filip Traverse', 'Damjan Chapman', 'Wulf Carlevaro', 'Aaron Dreschner', 'Kumar Dahl'
]

# Texto que se va a sintetizar
texto = "Hola! Soy el robot social Sancho, me gusta navegar por el laboratorio de MAPIR, y comer cables. ¡Adiós colega!"

# Crear carpeta si no existe
output_dir = "test"
os.makedirs(output_dir, exist_ok=True)

# Generar archivos de voz
for speaker in masculine_voices:
    output_path = os.path.join(output_dir, f"{speaker.replace(' ', '_')}.wav")
    print(f"Generando voz: {speaker}")
    try:
        tts.tts_to_file(
            text=texto,
            speaker=speaker,
            language="es",
            file_path=output_path
        )
    except Exception as e:
        print(f"Error con {speaker}: {e}")
