import os
import json
import random

from .ai import AI
from ..prompts.commands.commands import COMMANDS


class TemplateAI(AI):

    def __init__(self):
        current_dir = os.path.dirname(os.path.dirname(__file__))
        templates_path = os.path.join(current_dir, 'prompts', 'commands', 'templates.json')

        with open(templates_path, 'r', encoding='utf-8') as f:
            self.templates = json.load(f)

    def how_are_you(self):
        return random.choice(self.templates[COMMANDS.HOW_ARE_YOU])

    def what_you_see(self, actual_people):
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

        return random.choice(self.templates[COMMANDS.WHAT_YOU_SEE][group_key]).replace("{people}", people_str)

    def delete_user(self, user, result):
        return random.choice(self.templates[COMMANDS.DELETE_USER][result]).replace("{user}", user)
    
    def unknown_message(self):
        return random.choice(self.templates[COMMANDS.UNKNOWN])