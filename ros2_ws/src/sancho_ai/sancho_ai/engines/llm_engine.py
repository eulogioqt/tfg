"""TODO: Add module documentation."""
from .service_engine import ServiceEngine

from dotenv import load_dotenv

from llm_msgs.srv import Prompt


class LLMEngine(ServiceEngine):
"""TODO: Describe class."""
    def __init__(self, node):
    """TODO: Describe __init__.
Args:
    node (:obj:`Any`): TODO.
"""
        super().__init__(node)

        load_dotenv()

        self.prompt_cli = self.create_client(Prompt, "llm_tools/prompt")

        self.node.get_logger().info("Prompt Engine initializated succesfully")


    def prompt_request(self, provider="", model="", prompt_system="", messages_json="", user_input="", parameters_json=""):
    """TODO: Describe prompt_request.
Args:
    provider (:obj:`Any`): TODO.
    model (:obj:`Any`): TODO.
    prompt_system (:obj:`Any`): TODO.
    messages_json (:obj:`Any`): TODO.
    user_input (:obj:`Any`): TODO.
    parameters_json (:obj:`Any`): TODO.
"""
        req = Prompt.Request()
        req.provider = provider
        req.model = model
        req.prompt_system = prompt_system
        req.messages_json = messages_json
        req.user_input = user_input
        req.parameters_json = parameters_json

        result = self.call_service(self.prompt_cli, req)

        return result.response, result.provider_used, result.model_used, result.message, result.success
