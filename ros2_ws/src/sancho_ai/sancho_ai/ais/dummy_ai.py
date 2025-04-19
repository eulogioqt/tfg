from .ai import AI


class DummyAI(AI):

    def on_message(self, message):
        return message[::-1]