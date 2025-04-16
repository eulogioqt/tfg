from .ai import AI

class SimpleAI(AI):

    def on_message(self, message):
        return message[::-1]