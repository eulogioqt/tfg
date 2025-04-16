from .service_engine import ServiceEngine

from llm_msgs.srv import Prompt


class LLMEngine(ServiceEngine):
    def __init__(self, node):
        super().__init__(node)

        self.prompt_cli = self.create_client(Prompt, "llm_tools/prompt"),

        self.node.get_logger().info("Prompt Engine initializated succesfully")

    def prompt_request(self, provider="openai", model="gpt-3.5-turbo", prompt_system="", messages_json="", user_input="", parameters_json=""):
        req = Prompt.Request()
        req.provider = provider
        req.model = model
        req.prompt_system = prompt_system
        req.messages_json = messages_json
        req.user_input = user_input
        req.parameters_json = parameters_json

        result = self.call_service(self.prompt_cli, req)

        return result.response