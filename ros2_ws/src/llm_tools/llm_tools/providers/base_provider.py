"""TODO: Add module documentation."""
from abc import ABC, abstractmethod


class BaseProvider(ABC):

    @abstractmethod
"""TODO: Describe class."""
    def embedding(self, user_input, model):
    """TODO: Describe embedding.
Args:
    user_input (:obj:`Any`): TODO.
    model (:obj:`Any`): TODO.
"""
        pass

    @abstractmethod
    def prompt(self, user_input, model, prompt_system, messages_json, parameters_json): 
    """TODO: Describe prompt.
Args:
    user_input (:obj:`Any`): TODO.
    model (:obj:`Any`): TODO.
    prompt_system (:obj:`Any`): TODO.
    messages_json (:obj:`Any`): TODO.
    parameters_json (:obj:`Any`): TODO.
"""
        pass

    @abstractmethod
    def load(self, models):
    """TODO: Describe load.
Args:
    models (:obj:`Any`): TODO.
"""
        pass

    @abstractmethod
    def unload(self, models):
    """TODO: Describe unload.
Args:
    models (:obj:`Any`): TODO.
"""
        pass

    @abstractmethod
    def get_active_models(self):
    """TODO: Describe get_active_models.
"""
        pass
