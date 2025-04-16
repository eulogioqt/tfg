from abc import ABC, abstractmethod

class AI(ABC):

    @abstractmethod
    def on_message(self, message):
        pass
