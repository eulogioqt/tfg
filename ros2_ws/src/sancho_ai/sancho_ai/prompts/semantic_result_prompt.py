import json

from .prompt import Prompt
from .commands import CommandRegistry 

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


class SemanticResultPrompt(Prompt):
    def __init__(self, semantic_result: dict, user_input: str, chat_history: list = []):
        self.semantic_result = semantic_result
        self.user_input = user_input.strip()
        self.chat_history = chat_history
        self.intent_descriptions = self._load_intent_descriptions()

    def _load_intent_descriptions(self):
        return {cmd["name"]: cmd["description"] for cmd in CommandRegistry.get_commands()}

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

    @staticmethod
    def build_semantic_result(intent, arguments, status, details):
        return {
            "intent": intent,
            "arguments": arguments,
            "output": {
                "status": status,
                "details": details
            }
        }