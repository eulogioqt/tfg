import re
import json

from .template_ai import TemplateAI

from ..service_engine import ServiceEngine
from ..hri_engine import HRIEngine
from ..llm_engine import LLMEngine

from ..prompts.classification_prompt import ClassificationPrompt
from ..prompts.commands.commands import COMMANDS

def try_json_loads(text):
    try:
        return json.loads(text)
    except Exception:
        return None

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
            prompt_system=classification_prompt.get_prompt_system(),
            user_input=classification_prompt.get_user_prompt(),
            parameters_json=classification_prompt.get_parameters()
        )
        
        if not success:
            self.node.get_logger().error(f"There was a problem with classification prompt: {message}")
            return self.unknown_message(), COMMANDS.UNKNOWN, provider_used, model_used
        
        self.node.get_logger().info(f"LLM:\n{response}")
        classification_response_json = extract_json_from_code_block(response) # Gemini usually puts the response in ```json block
        if not classification_response_json:
            self.node.get_logger().error(f"No JSON format found.")
            return self.unknown_message(), COMMANDS.UNKNOWN, provider_used, model_used

        classification_response = try_json_loads(classification_response_json)
        if not classification_response:
            self.node.get_logger().error(f"Error on JSON loads.")
            return self.unknown_message(), COMMANDS.UNKNOWN, provider_used, model_used
        
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

        return response, intent, provider_used, model_used