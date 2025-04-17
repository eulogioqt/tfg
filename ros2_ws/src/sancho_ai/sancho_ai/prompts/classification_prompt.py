import os
import json

from .prompt import Prompt

PROMPT_TEMPLATE = """
You are an intent classifier for a conversational robot.
Your task is to analyze the user's input and return a JSON object in this format:

{
  "intent": string,   // One of the intent codes listed below, or "UNKNOWN"
  "arguments": object // Relevant arguments (if any), or an empty object {}
}

Valid intents:
{intents_definitions}

If the input doesn't match any known intent, respond with:
{
  "intent": "UNKNOWN",
  "arguments": {}
}

Examples:
{examples_section}

Important: Only return valid JSON. No explanations. No markdown. No extra text.
"""

class ClassificationPrompt(Prompt):
    def __init__(self, user_input: str):
        self.user_input = user_input

        current_dir = os.path.dirname(__file__)
        commands_path = os.path.join(current_dir, 'commands', 'commands.json')

        with open(commands_path, 'r', encoding='utf-8') as f:
            self.commands = json.load(f)

        self.intents_definitions_str = self._format_intents()
        self.examples_str = self._format_examples()

    def _format_intents(self):
        lines = []
        for cmd in self.commands:
            line = f"- {cmd['name']}: {cmd['description']}"
            args = cmd.get("arguments", {})
            if args:
                arg_list = ", ".join([f'"{arg}": {desc}' for arg, desc in args.items()])
                line += f" (arguments: {{{arg_list}}})"
            lines.append(line)
        return "\n".join(lines)

    def _format_examples(self):
        examples = []
        for cmd in self.commands:
            for example in cmd.get("examples", []):
                input_text = example["input"]
                output_json = json.dumps(example["output"], ensure_ascii=False)
                examples.append(f'Input: "{input_text}"\nOutput: {output_json}')
 
        examples.append('Input: "me gusta mucho el fútbol"\nOutput: {"intent": "UNKNOWN", "arguments": {}}')
        return "\n\n".join(examples)

    def get_prompt_system(self):
        return PROMPT_TEMPLATE.replace("{intents_definitions}", self.intents_definitions_str)\
                              .replace("{examples_section}", self.examples_str)

    def get_user_prompt(self):
        return self.user_input

    def get_parameters(self):
        return json.dumps({
            "temperature": 0.0,
            "max_tokens": 512
        })
