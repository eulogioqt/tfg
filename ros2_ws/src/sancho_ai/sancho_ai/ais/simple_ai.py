from .ai import AI

class SimpleAI(AI):

    def __init__(self):
        super().__init__()
    
    def on_message(self, message):
        return message[::-1]