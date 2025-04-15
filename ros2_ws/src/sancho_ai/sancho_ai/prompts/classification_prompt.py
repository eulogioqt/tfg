import os
import json

from .prompt import Prompt

# Mejorar esto a futuro, en ingles y demas, no tan simple, estudiarlo bien, ahora solo estoy probando
PROMPT_TEMPLATE = """
Eres un clasificador de intenciones para un robot conversacional.
Tu tarea es analizar una frase del usuario y devolver un objeto JSON con esta estructura:

{{
  "intent": string,  // uno de los códigos de intención definidos abajo o "UNKNOWN"
  "arguments": dict  // los argumentos que correspondan o un objeto vacío
}}

Estas son las intenciones válidas y su definición:
{intents_definitions}

Tu respuesta debe ser SIEMPRE un JSON. Si no reconoces ninguna intención, pon "intent": "UNKNOWN".
No expliques nada. Solo responde el JSON sin formato adicional.
"""

class ClassificationPrompt(Prompt):
    def __init__(self, user_input: str):
        self.user_input = user_input

        current_dir = os.path.dirname(__file__)
        commands_path = os.path.join(current_dir, '..', '..', 'commands', 'commands.json')

        with open(commands_path, 'r', encoding='utf-8') as f:
            self.intents_definitions_data = json.load(f)

        self.intents_definitions_str = json.dumps(
            self.intents_definitions_data,
            indent=2,
            ensure_ascii=False
        )

    def get_prompt_system(self):
        return PROMPT_TEMPLATE.replace("{intents_definitions}", self.intents_definitions_str)

    def get_user_prompt(self):
        return f"Usuario: {self.user_input}"

    def get_parameters(self):
        return {
            "temperature": 0.0,
            "max_tokens": 1024 # De sobra para que de el JSON bien
        }
