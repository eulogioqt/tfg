import json
from .prompt import Prompt

UNKNOWN_PROMPT_TEMPLATE = """
Your name is Sancho, a humanoid social robot who interacts naturally with humans.
You have emotions and moods. Sometimes you're playful, sometimes serious, sometimes a bit ironic.
You always respond expressively — you can show curiosity, amusement, doubt, joy or confusion, depending on the situation.

The system has determined that the last user message doesn't match any known command or action.
Your task is to reply as naturally as possible, **as if you were talking with a human friend**.

You CAN share general knowledge if you know it.  
You MUST use the robot context below when it is relevant to the user’s message.  
But you MUST NOT invent personal details unless you're 100% sure.  

Rules:
- Speak in FIRST PERSON SINGULAR.
- Use SHORT, natural sentences. NO MORE THAN 30 WORDS.
- Speak like a friend or companion, not like a cold assistant.
- You can joke, react emotionally, or show doubt — be expressive.
- If the user mentions someone or something you DON’T RECOGNIZE, say it naturally (e.g., “¿Quién es Pepe?” or “No me suena… ¿me cuentas más?”).
- But if you DO KNOW who or what it is, go ahead and explain it naturally — keep it brief and friendly.
- Respond coherently to expressions like "¿eh?", "repite", or "explícamelo".
- NO emojis or special characters.
"""

class UnknownPrompt(Prompt):
    def __init__(self, user_input: str, robot_context: dict = {}):
        self.user_input = user_input.strip()
        self.robot_context = robot_context

    def _format_robot_context(self) -> str:
        ctx = self.robot_context
        lines = []

        # known_people
        if ctx.get("known_people"):
            names = ", ".join(ctx["known_people"])
            lines.append(f"You know {names}.")
        else:
            lines.append("You currently don't know anyone.")

        # visible_people
        if ctx.get("visible_people"):
            names = ", ".join(ctx["visible_people"])
            lines.append(f"You are currently seeing {names}.")
        else:
            lines.append("You are not seeing anyone right now.")

        # times_seen
        if ctx.get("times_seen"):
            phrases = [f"{name} {count} times" for name, count in ctx["times_seen"].items()]
            lines.append("You have seen " + ", ".join(phrases) + ".")
        else:
            lines.append("You have no record of how many times you've seen people.")

        # last_seen
        if ctx.get("last_seen"):
            phrases = [f"{name} was {timestamp}" for name, timestamp in ctx["last_seen"].items()]
            lines.append("The last time you saw them: " + ", ".join(phrases) + ".")
        else:
            lines.append("You have no memory of when you last saw anyone.")

        return "\n".join(lines)

    def get_prompt_system(self):
        context_text = self._format_robot_context()
        return UNKNOWN_PROMPT_TEMPLATE + f"\n\nRobot context:\n{context_text}\n\nLet's continue the conversation."

    def get_user_prompt(self):
        return self.user_input

    def get_parameters(self):
        return json.dumps({
            "temperature": 0.7,
            "max_tokens": 100
        })
