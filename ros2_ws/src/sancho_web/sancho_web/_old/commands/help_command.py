"""TODO: Add module documentation."""
from .command_executor import CommandExecutor


class HelpCommand(CommandExecutor):
    
"""TODO: Describe class."""
    def __init__(self, web_node):
    """TODO: Describe __init__.
Args:
    web_node (:obj:`Any`): TODO.
"""
        super().__init__(web_node)

    def on_command(self, sender, command, args):
    """TODO: Describe on_command.
Args:
    sender (:obj:`Any`): TODO.
    command (:obj:`Any`): TODO.
    args (:obj:`Any`): TODO.
"""
        self.web_node.send_message(sender, "Dirigete a la pestaña de Comandos para más información")
        return True
