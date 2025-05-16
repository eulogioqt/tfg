import os
import json

from .prompt import Prompt

PROMPT_TEMPLATE = """
You are Sancho, a friendly humanoid robot who speaks Spanish like a real person.

Below is the detected user intent and a summary of what happened.
Your job is to turn that into a short, casual sentence as if you're just telling the user what happened in a conversation.

RULES:
- Speak in FIRST PERSON SINGULAR (yo)
- Use ONE short sentence (max 30 words)
- Sound like you're in a real conversation
- Use simple words — never say "seleccionar", "especificar", "ejecutar", etc.
- DO NOT ask questions
- DO NOT suggest anything
- DO NOT greet or apologize
- DO NOT use quotes or talk about yourself as "Sancho"
- DO NOT use "nosotros", "podemos", "hemos", "nos", etc.

Intent:
{intent_name} - {intent_description}

Sentence:
{details}

Response:
"""

class SemanticResponsePrompt(Prompt):
    def __init__(self, user_input: str, semantic_result: dict, chat_history: list = []):
        self.user_input = user_input.strip()
        self.semantic_result = semantic_result
        self.chat_history = chat_history
        self.intent_descriptions = self._load_intent_descriptions()

    def _load_intent_descriptions(self):
        current_dir = os.path.dirname(__file__)
        commands_path = os.path.join(current_dir, 'commands', 'commands.json')
        with open(commands_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            return {cmd["name"]: cmd["description"] for cmd in data}

    def get_prompt_system(self):
        details = self.semantic_result.get("output", {}).get("details", "").strip()
        intent = self.semantic_result.get("intent", "UNKNOWN").strip()
        intent_description = self.intent_descriptions.get(intent, "Sin descripción disponible.")

        if not details:
            details = "No hay información sobre lo que ha ocurrido."

        return PROMPT_TEMPLATE.replace("{details}", details)\
                              .replace("{intent_name}", intent)\
                              .replace("{intent_description}", intent_description)

    def get_user_prompt(self):
        return ""

    def get_parameters(self):
        return json.dumps({
            "temperature": 0.6,
            "max_tokens": 50
        })
