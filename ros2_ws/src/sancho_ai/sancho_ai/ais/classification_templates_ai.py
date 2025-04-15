import os
import json
import random

from .ai import AI
from ..prompt_engine import PromptEngine
from ..prompts.classification_prompt import ClassificationPrompt
from ..prompts.commands.commands import COMMANDS

class ClassificationTemplatesAI(AI):

    def __init__(self):
        super().__init__()

        self.engine = PromptEngine(self.node)
        
        current_dir = os.path.dirname(__file__)
        templates_path = os.path.join(current_dir, '..', '..', 'prompts', 'commands', 'templates.json')

        with open(templates_path, 'r', encoding='utf-8') as f:
            self.templates = json.load(f)

    def on_message(self, message):
        classification_prompt = ClassificationPrompt(message)
        print(classification_prompt.get_prompt_system())

        classification_response_json = self.engine.prompt_request(
            prompt_system=classification_prompt.get_prompt_system(),
            user_input=classification_prompt.get_user_prompt(),
            parameters_json=classification_prompt.get_parameters()
        )

        print(classification_response_json)
        classification_response = json.loads(classification_response_json)
        intent = classification_response["intent"]

        if intent == COMMANDS.HOW_ARE_YOU:
            response = random.choice(self.templates[intent])
        elif intent == COMMANDS.WHAT_YOU_SEE:
            people = "Sebas y Guille"

            response = random.choice(self.templates[intent]).replace("{people}", people)
        elif intent == COMMANDS.DELETE_USER:
            result = random.choice(["failure", "success"])
            user = "Josemi"

            response = random.choice(self.templates[result][intent]).replace("{user}", user)
        else:
            response = random.choice(self.templates["UNKNOWN"])

        return response