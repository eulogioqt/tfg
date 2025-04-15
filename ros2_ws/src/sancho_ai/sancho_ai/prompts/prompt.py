from abc import ABC, abstractmethod

class Prompt(ABC):

    @abstractmethod
    def get_prompt_system(self):
        pass

    @abstractmethod
    def get_user_prompt(self):
        pass

    @abstractmethod
    def get_parameters(self):
        pass
