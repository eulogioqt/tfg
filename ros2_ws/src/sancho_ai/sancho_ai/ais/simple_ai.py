import json

from .template_ai import TemplateAI

from ..service_engine import ServiceEngine
from ..hri_engine import HRIEngine

class SimpleAI(TemplateAI):

    def __init__(self):
        super().__init__()

        self.node = ServiceEngine.create_client_node()
        self.hri_engine = HRIEngine(self.node)

    def on_message(self, message: str):
        if 'como estas' in message:  # Busca en Wikipedia lo que le digas
            response = self.how_are_you()
        elif 'quien soy' == message or 'que ves' in message or 'quien ves' in message:
            actual_people_json = self.hri_engine.get_actual_people_request()
            actual_people = json.loads(actual_people_json)

            response = self.what_you_see(actual_people)
        elif message.startswith('borra a'):
            user = message[(len("borra a") + 1):]
            result = self.hri_engine.delete_request(user)
            result = "success" if result >= 0 else "failure"

            response = self.delete_user(user, result)
        else: 
            response = self.unknown_message()
        
        return response

