import re
import json

from .template_ai import TemplateAI

from ..service_engine import ServiceEngine
from ..hri_engine import HRIEngine
from ..llm_engine import LLMEngine

from ..prompts.classification_prompt import ClassificationPrompt
from ..prompts.commands.commands import COMMANDS

from llm_tools.models import PROVIDER, MODELS


def extract_json_from_code_block(text):
    match = re.search(r"```json\s*(\{.*?\})\s*```", text, re.DOTALL)
    if match:
        return match.group(1).strip()
    
    match = re.search(r"(\{.*\})", text, re.DOTALL)
    if match:
        return match.group(1).strip()
    
    raise None


class ClassificationTemplatesAI(TemplateAI):

    def __init__(self):
        super().__init__()

        self.node = ServiceEngine.create_client_node()
        self.hri_engine = HRIEngine(self.node)
        self.llm_engine = LLMEngine(self.node)
    
    def on_message(self, user_input):
        classification_prompt = ClassificationPrompt(user_input)

        self.node.get_logger().info(f"User: {user_input}")
        response, provider_used, model_used, message, success = self.llm_engine.prompt_request(
            #provider=PROVIDER.GEMINI, # que esto devuelva el provider y el modelo por si quiero ponerlo por ahi en la web
            #model=MODELS.LLM.DEEPSEEK.DEEPSEEK_CHAT,
            prompt_system=classification_prompt.get_prompt_system(),
            user_input=classification_prompt.get_user_prompt(),
            parameters_json=classification_prompt.get_parameters()
        )
        
        if not success:
            self.node.get_logger().error(f"There was a problem with prompt: {message}")
            return self.unknown_message()
        
        classification_response_json = extract_json_from_code_block(response) # Gemini usually puts the response in ```json block
        if not classification_response_json:
            self.node.get_logger().error(f"No JSON format: {classification_response_json}")
            return self.unknown_message()

        self.node.get_logger().info(f"Assistant: {classification_response_json}")
        classification_response = json.loads(classification_response_json)
        intent = classification_response["intent"]

        if intent == COMMANDS.HOW_ARE_YOU:
            response = self.how_are_you()
        elif intent == COMMANDS.WHAT_YOU_SEE:
            actual_people_json = self.hri_engine.get_actual_people_request()
            actual_people = json.loads(actual_people_json)

            response = self.what_you_see(actual_people)
        elif intent == COMMANDS.DELETE_USER:
            user = classification_response["arguments"]["user"]
            result = self.hri_engine.delete_request(user)
            result = "success" if result >= 0 else "failure"

            response = self.delete_user(user, result)
        else:
            response = self.unknown_message()

        return response