"""TODO: Add module documentation."""
from hri_msgs.msg import BodyMovement
from std_msgs.msg import String, Bool

from .command_executor import CommandExecutor


class MovebodyCommand(CommandExecutor):
    
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
        if len(args) == 0:
            self.web_node.send_message(sender, "No has especificado la distancia mov_x")
        else:
            if not is_float(args[0]):
                self.web_node.send_message(sender, "&l" + args[0] + "&0 no es una distancia valida")
            else:
                mov_x, mode = float(args[0]), "absolute"
                if len(args) > 1:
                    mode = args[1]

                if mode not in ["absolute", "relative"]:
                    self.web_node.send_message(sender, "&l" + mode + "&0 no es un modo valido. Debe ser &l'absolute'&0 o &l'relative'")     
                else:
                    self.web_node.node.robot_movement.publish(BodyMovement(mode=String(data=mode), force=Bool(data=True), mov_x=mov_x))
                    if mode == "absolute":
                        self.web_node.send_message(sender, "Moviendo el cuerpo a &l" + str(mov_x)) 
                    elif mode == "relative":
                        self.web_node.send_message(sender, "Moviendo el cuerpo &l" + str(mov_x) + "&0 metros respecto a su posición actual") 

        return True

def is_float(str):
"""TODO: Describe is_float.
Args:
    str (:obj:`Any`): TODO.
"""
    try:
        float(str)
        return True
    except Exception:
        return False
