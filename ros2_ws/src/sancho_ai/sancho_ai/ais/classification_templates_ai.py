import os
import json
import random

from .ai import AI
from ..service_engine import ServiceEngine
from ..prompts.classification_prompt import ClassificationPrompt
from ..prompts.commands.commands import COMMANDS

class ClassificationTemplatesAI(AI):

    def __init__(self):
        self.engine = ServiceEngine()
        
        current_dir = os.path.dirname(os.path.dirname(__file__))
        templates_path = os.path.join(current_dir, 'prompts', 'commands', 'templates.json')

        with open(templates_path, 'r', encoding='utf-8') as f:
            self.templates = json.load(f)

    def on_message(self, message):
        classification_prompt = ClassificationPrompt(message)

        classification_response_json = self.engine.prompt_request(
            prompt_system=classification_prompt.get_prompt_system(),
            user_input=classification_prompt.get_user_prompt(),
            parameters_json=classification_prompt.get_parameters()
        )

        classification_response = json.loads(classification_response_json)
        intent = classification_response["intent"]

        if intent == COMMANDS.HOW_ARE_YOU:
            response = random.choice(self.templates[intent])
        elif intent == COMMANDS.WHAT_YOU_SEE:
            actual_people_json = self.engine.get_actual_people_request()
            actual_people = json.loads(actual_people_json)

            people_on_screen = [person for person, time_on_screen in actual_people.items() if time_on_screen < 1]
            if not people_on_screen:
                people_str = ""
                group_key = "none"
            elif len(people_on_screen) == 1:
                people_str = people_on_screen[0]
                group_key = "one"
            else:
                people_str = ", ".join(people_on_screen[:-1]) + f" y {people_on_screen[-1]}"
                group_key = "many"

            response = random.choice(self.templates[intent][group_key]).replace("{people}", people_str)
        elif intent == COMMANDS.DELETE_USER:
            result = random.choice(["failure", "success"])
            user = classification_response["arguments"]["user"]

            response = random.choice(self.templates[intent][result]).replace("{user}", user)
        else:
            response = random.choice(self.templates["UNKNOWN"])

        return response