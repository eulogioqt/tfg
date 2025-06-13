"""TODO: Add module documentation."""
import os
import json

class CommandRegistry:
"""TODO: Describe class."""
    _commands = None

    @classmethod
    def get_commands(cls):
    """TODO: Describe get_commands.
Args:
    cls (:obj:`Any`): TODO.
"""
        if cls._commands is None:
            current_dir = os.path.dirname(__file__)
            path = os.path.join(current_dir, 'commands.json')
            with open(path, 'r', encoding='utf-8') as f:
                cls._commands = json.load(f)
        return cls._commands
