import os
import json
import openai

from dotenv import load_dotenv

load_dotenv()

client = openai.OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

INTENTS_DEFINITIONS = """
[
  {
    "name": "BORRAR_USUARIO",
    "description": "Eliminar o borrar a un usuario del sistema.",
    "arguments": {
      "nombre_usuario": "Nombre del usuario a eliminar (si se menciona)."
    }
  },
  {
    "name": "QUE_VES",
    "description": "Saber qué está observando el robot o qué se ve por la cámara.",
    "arguments": {}
  },
  {
    "name": "COMO_ESTAS",
    "description": "Conocer el estado emocional o físico del robot.",
    "arguments": {}
  }
]
"""

PROMPT_SYSTEM = f"""
Eres un clasificador de intenciones para un robot conversacional.
Tu tarea es analizar una frase del usuario y devolver un objeto JSON con esta estructura:

{{
  "intent": string,  // uno de los códigos de intención definidos abajo o "NINGUNO"
  "arguments": dict  // los argumentos que correspondan o un objeto vacío
}}

Estas son las intenciones válidas y su definición:
{INTENTS_DEFINITIONS}

Tu respuesta debe ser SIEMPRE un JSON. Si no reconoces ninguna intención, pon "intent": "NINGUNO".
No expliques nada. Solo responde el JSON sin formato adicional.
"""

def classify_and_extract(user_input: str) -> dict:
    messages = [
        {"role": "system", "content": PROMPT_SYSTEM},
        {"role": "user", "content": f'Usuario: "{user_input}"'}
    ]

    response = client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=messages,
        temperature=0.0
    )

    content = response.choices[0].message.content
    try:
        return json.loads(content)
    except Exception as e:
        print("❌ Error al interpretar JSON:", e)
        print("🔎 Respuesta LLM:", content)
        return {"intent": "ERROR", "arguments": {}}

# 🧪 Ejemplo
while True:
    print(PROMPT_SYSTEM)
    
    inp = input("Frase: ")
    if inp.lower() == "salir":
        break
    resultado = classify_and_extract(inp)
    print("👉 Clasificado como:", resultado)
