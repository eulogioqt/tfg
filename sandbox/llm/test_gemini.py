import os
import google.generativeai as genai
from dotenv import load_dotenv

# Cargar API key desde .env
load_dotenv()
api_key = os.getenv("GEMINI_API_KEY")

if not api_key:
    raise ValueError("No se encontró la clave GEMINI_API_KEY en .env")

genai.configure(api_key=api_key)

# Prioridad de modelos (más potentes primero)
PREFERRED_MODELS = [
    "gemini-1.5-pro",
    "gemini-1.5-pro-latest",
    "gemini-1.5-flash",
    "gemini-1.5-flash-latest"
]

def get_available_models():
    try:
        models = genai.list_models()
        available = [
            m.name for m in models
            if "generateContent" in m.supported_generation_methods
        ]
        return available
    except Exception as e:
        print("Error listando modelos:", e)
        return []

def try_prompt_with_model(model_name, prompt):
    try:
        print(f"\n=== Probando modelo: {model_name} ===")
        model = genai.GenerativeModel(model_name)
        response = model.generate_content(prompt)
        print("Respuesta:")
        print(response.text)
        return True
    except Exception as e:
        print(f"Error con {model_name}:\n{e}\n")
        return False

def main():
    prompt = "Resume en una frase qué es la inteligencia artificial."
    available_models = get_available_models()

    print("Modelos disponibles:", available_models)

    for model in PREFERRED_MODELS:
        if model in available_models:
            success = try_prompt_with_model(model, prompt)
            if success:
                break
    else:
        print("❌ Ningún modelo disponible o con cuota libre.")

if __name__ == "__main__":
    main()
