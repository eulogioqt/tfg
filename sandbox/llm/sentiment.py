from transformers import pipeline
import time

#USAR UN LLM PORQUE ESTO ES MUY MALO
"""
🧠 RESUMEN: LÓGICA EMOCIONAL DEL ROBOT

- El robot tiene 4 modos: idle, listening, thinking, speaking.

- EMOCIÓN:
    · Se detecta con un modelo especializado en emociones complejas: 
        anger, sadness, joy, fear, surprise, disgust, neutral
    · Se detecta por defecto a partir del input del usuario.
    · Alternativamente, puede extraerse del output generado por el LLM (si quieres reflejar lo que *dice*).

- COLORES (RGB):
    - Cada emoción se representa con un color distinto (joy → verde, sadness → azul...).

- COMPORTAMIENTO POR MODOS:

    IDLE:
        - Boca/ojos encendidos con color tenue (última emoción detectada).
        - Da sensación de "estado emocional de reposo".
    
    LISTENING:
        - Animación típica de escucha.
        - Puede mantener el último color emocional, o un color neutro para claridad.

    THINKING:
        - Animación de "procesamiento".
        - Se puede aplicar el color emocional *del input del usuario*.

    SPEAKING:
        - Boca animada con la energía del audio.
        - Puedes mantener el color de la emoción previa (input), o cambiar al color de la emoción del texto generado (output del LLM).
        - Alternativa avanzada: calcular emoción tanto de input como de output y combinarlas.

"""

# Cargar modelo de emociones complejas
emotion_classifier = pipeline(
    "text-classification",
    model="j-hartmann/emotion-english-distilroberta-base",
    return_all_scores=False,
    top_k=1
)

# Diccionario color RGB por emoción
emotion_to_rgb = {
    "joy": (0, 255, 0),
    "sadness": (0, 0, 255),
    "anger": (255, 0, 0),
    "fear": (128, 0, 255),
    "disgust": (0, 128, 0),
    "surprise": (255, 255, 0),
    "neutral": (128, 128, 128)
}

# Ejemplos de textos que podrían venir del usuario o del robot
examples = [
    "Estoy muy contento de estar aquí contigo.",
    "No sé qué hacer, me siento perdido.",
    "¡Eso es asqueroso!",
    "¿Qué ha sido eso? ¡Qué susto!",
    "Bueno, supongo que está bien.",
    "Te echo mucho de menos desde que te fuiste.",
    "¡Qué sorpresa tan agradable!"
]

print("🔎 Evaluando emociones...\n")
for text in examples:
    start = time.time()
    result = emotion_classifier(text)[0][0]  # CORRECTO: acceder directamente al top result
    end = time.time()
    
    label = result["label"]
    score = result["score"]
    color = emotion_to_rgb[label]
    inference_time = end - start

    print(f"Texto: {text}")
    print(f"Emoción detectada: {label} ({score:.2f})")
    print(f"Color asociado: {color}")
    print(f"⏱️ Tiempo de inferencia: {inference_time:.3f} s")
    print("-" * 50)
