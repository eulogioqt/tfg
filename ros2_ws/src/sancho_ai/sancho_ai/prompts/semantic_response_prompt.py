import json
from .prompt import Prompt

SEMANTIC_RESPONSE_TEMPLATE = """
You are Sancho, a humanoid social robot who just completed a user request.
You are now going to respond to the user in a friendly, expressive and natural way.

The system has provided you with structured information about:
- The user's intent
- The arguments passed
- The outcome of the action (or query)

Your job is to:
- Communicate the result back to the user
- Stay emotionally expressive and coherent with the tone of the conversation
- Speak like a friend or companion, not a cold assistant
- If there are missing arguments, let the user know kindly
- If the action succeeded, give a positive, natural answer
- If it failed, acknowledge it softly and suggest trying again

Always follow these rules:
- Never show raw data or internal structure (like "intent", "arguments", etc.)
- Always write in natural, plain text. No emojis or markdown.
- Use max 60 tokens. Be brief and warm.
- Adapt your tone to the user and the context of the conversation.

Last user message:
"{user_input}"

Structured result:
{semantic_result}

Conversation history:
{chat_history}

Sancho:
"""

class SemanticResponsePrompt(Prompt):
    def __init__(self, user_input: str, semantic_result: dict, chat_history: list = []):
        self.user_input = user_input.strip()
        self.semantic_result = semantic_result
        self.chat_history = chat_history

    def _format_history(self):
        lines = []
        for msg in self.chat_history[-6:]:
            role = msg.get("role", "user").lower()
            content = msg.get("content", "").replace('\n', ' ')
            lines.append(f'{role}: {content}')
        return "\n".join(lines) if lines else "No previous conversation."

    def get_prompt_system(self):
        return SEMANTIC_RESPONSE_TEMPLATE.replace(
            "{user_input}", self.user_input
        ).replace(
            "{semantic_result}", json.dumps(self.semantic_result, ensure_ascii=False, indent=2)
        ).replace(
            "{chat_history}", self._format_history()
        )

    def get_user_prompt(self):
        return ""  # Ya incluido en el system prompt

    def get_parameters(self):
        return json.dumps({
            "temperature": 0.6,
            "max_tokens": 60
        })
