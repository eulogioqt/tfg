import rclpy
from rclpy.node import Node

from hri_msgs.srv import SanchoPrompt

from .ais.factory import create_sancho_ai, AIType

# meter comandos de dime cual es la ultima persona que has visto
# de que diga cuanta gente conoce
# lo ideal seguramente sea en el contexto meterle toda esta info si son demasiados comandos y dejar comando solo para lo que haga acciones

# hacer que haya un diccionario con el comando y la funcion y luego solo se pasan los argumentos con ** y ea, y que el simple ai tambien
# devuelva las cosas en el mismo formato 
# que el llm y ya zabe picha awa niña esto es porti
# entonces dado un prompt, el on command que se llame de otra forma o algo y lo que hace es clasificar la intencion
# devuelve en json el formato ese asin epico y luego ya con el diccionario se ejecuta la funcion y depende de la respuesta po una coza o otra

# ahora esta el template ai pero se pueden hacer otras o que a esta puedas pasarle el template tu opcionalmente o cosas asin o que las funciones
# respondan con otro json y yo ese json ya lo uso para crear la respuesta o lo que sea, mas general sabe, me mola asi esta forma, mas parecido al MCP
class SanchoAINode(Node):

    def __init__(self, type):
        super().__init__("sancho_ai")

        self.sancho_ai = create_sancho_ai(type)
        self.prompt_serv = self.create_service(SanchoPrompt, "sancho_ai/prompt", self.prompt_service)

        self.get_logger().info("SanchoAI Node initializated succesfully")

    def prompt_service(self, request, response):
        response.text = self.sancho_ai.on_message(request.text)
        return response

def main(args=None):
    rclpy.init(args=args)

    node = SanchoAINode(AIType.CLASSIFICATION_TEMPLATES)

    rclpy.spin(node)
    rclpy.shutdown()