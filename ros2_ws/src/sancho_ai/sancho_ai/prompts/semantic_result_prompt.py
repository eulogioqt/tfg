"""TODO: Add module documentation."""
import json

from .prompt import Prompt
from .commands import CommandRegistry 

PROMPT_TEMPLATE = """
Your name is Sancho. You are a friendly and expressive humanoid robot who speaks **only in Spanish**.

Below is the user intent and a technical summary of what happened.

Your task is to **rephrase the message** in a natural, casual tone — as if you were just telling someone what happened.  
Imagine you're speaking out loud, like a real human would do.

You also need to decide what **emotion** you (Sancho) should express when saying this.  
This will affect your facial expression and tone.

Rules:
- Response must be in SPANISH.
- Speak in FIRST PERSON SINGULAR (yo).
- Use **ONE sentence only**, with a maximum of 30 words.
- Keep it simple, expressive and human — as if you're in a casual conversation.
- You can sound confident, annoyed, relaxed, happy, etc.
- DO NOT say "Sancho", "nosotros", "podemos", "ejecutar", "procedimiento", etc.
- DO NOT ask questions, greet, apologize or suggest further actions.

You must return a JSON like this:
{
  "response": "your full sentence in Spanish",
  "emotion": "happy | sad | angry | bored | suspicious | neutral"
}

Intent:
{intent_name} - {intent_description}

Original message:
{details}

Now rewrite the message in a natural way and choose an appropriate emotion.
Return only the JSON.
"""

class SemanticResultPrompt(Prompt):
"""TODO: Describe class."""
    def __init__(self, semantic_result: dict, user_input: str, chat_history: list = []):
    """TODO: Describe __init__.
Args:
    semantic_result (:obj:`Any`): TODO.
    user_input (:obj:`Any`): TODO.
    chat_history (:obj:`Any`): TODO.
"""
        self.semantic_result = semantic_result
        self.user_input = user_input.strip()
        self.chat_history = chat_history
        self.intent_descriptions = self._load_intent_descriptions()

    def _load_intent_descriptions(self):
    """TODO: Describe _load_intent_descriptions.
"""
        return {cmd["name"]: cmd["description"] for cmd in CommandRegistry.get_commands()}

    def get_prompt_system(self):
    """TODO: Describe get_prompt_system.
"""
        details = self.semantic_result.get("output", {}).get("details", "").strip()
        intent = self.semantic_result.get("intent", "UNKNOWN").strip()
        intent_description = self.intent_descriptions.get(intent, "Sin descripción disponible.")

        if not details:
            details = "No hay información sobre lo que ha ocurrido."

        return PROMPT_TEMPLATE.replace("{details}", details)\
                              .replace("{intent_name}", intent)\
                              .replace("{intent_description}", intent_description)

    def get_user_prompt(self):
    """TODO: Describe get_user_prompt.
"""
        return ""

    def get_parameters(self):
    """TODO: Describe get_parameters.
"""
        return json.dumps({
            "temperature": 0.65,
            "max_tokens": 50
        })

    @staticmethod
    def build_semantic_result(intent, arguments, status, details):
    """TODO: Describe build_semantic_result.
Args:
    intent (:obj:`Any`): TODO.
    arguments (:obj:`Any`): TODO.
    status (:obj:`Any`): TODO.
    details (:obj:`Any`): TODO.
"""
        return {
            "intent": intent,
            "arguments": arguments,
            "output": {
                "status": status,
                "details": details
            }
        }
