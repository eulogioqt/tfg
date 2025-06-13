"""TODO: Add module documentation."""
from abc import ABC, abstractmethod


class CommandExecutor(ABC):
    
"""TODO: Describe class."""
    def __init__(self, web_node):
    """TODO: Describe __init__.
Args:
    web_node (:obj:`Any`): TODO.
"""
        self.web_node = web_node

    @abstractmethod
    def on_command(self, sender, command, args):
    """TODO: Describe on_command.
Args:
    sender (:obj:`Any`): TODO.
    command (:obj:`Any`): TODO.
    args (:obj:`Any`): TODO.
"""
        pass

    @staticmethod
    def equals_ignore_case(str1, str2):
    """TODO: Describe equals_ignore_case.
Args:
    str1 (:obj:`Any`): TODO.
    str2 (:obj:`Any`): TODO.
"""
        return str1.lower() == str2.lower()
