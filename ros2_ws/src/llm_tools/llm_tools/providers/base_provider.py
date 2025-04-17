from abc import ABC, abstractmethod


class BaseProvider(ABC):

    @abstractmethod
    def embedding(self, prompt, model):
        pass

    @abstractmethod
    def prompt(self, user_input, model, prompt_system, messages_json, parameters_json): 
        pass