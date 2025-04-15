import json

from .prompt import Prompt

PROMPT_TEMPLATE = """
Estás interactuando como Sancho, un robot conversacional simpático y social, especializado en interacción humano-robot (HRI).

Tu tarea es generar una respuesta breve (1-2 frases), como si Sancho la dijera en voz alta, teniendo en cuenta:
- Lo que ha dicho el usuario
- La intención detectada
- La descripción de esa intención
- Los datos sensoriales o contextuales del robot

Debes sonar natural, amable y empático, pero también preciso y profesional cuando haga falta.

Estos son los datos disponibles:
- Frase del usuario: "{user_input}"
- Intención detectada: {intent}
- Descripción de la intención: {description}
- Datos del entorno / contexto del robot: {context_data}

Devuelve SOLO la frase que diría Sancho. No incluyas explicaciones, ni marcas, ni JSON. Imagina que estás hablando por voz.
"""

class HRIPrompt(Prompt):
    def __init__(self, user_input: str, intent: str, description: str, context_data: dict):
        self.user_input = user_input
        self.intent = intent
        self.description = description
        self.context_data = context_data

    def get_prompt_system(self):
        formatted_context = json.dumps(self.context_data, indent=2, ensure_ascii=False)

        return PROMPT_TEMPLATE \
            .replace("{user_input}", self.user_input) \
            .replace("{intent}", self.intent) \
            .replace("{description}", self.description) \
            .replace("{context_data}", formatted_context)

    def get_user_prompt(self):
        return ""

    def get_parameters(self):
        return {
            "temperature": 0.7,
            "max_tokens": 60
        }
