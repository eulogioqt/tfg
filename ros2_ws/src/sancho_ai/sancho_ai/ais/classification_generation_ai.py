import re
import json

from .generate_ai import GenerateAI

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


class ClassificationGenerationAI(GenerateAI):

    def __init__(self):
        super().__init__()

        self.node = ServiceEngine.create_client_node()
        self.hri_engine = HRIEngine(self.node)
        self.llm_engine = LLMEngine(self.node)
    
    def on_message(self, user_input, chat_history=[]):
        classification_prompt = ClassificationPrompt(user_input, chat_history) # Ultimos 5 turnos

        self.node.get_logger().info(f"User: {user_input}")
        response, provider_used, model_used, message, success = self.llm_engine.prompt_request(
            prompt_system=classification_prompt.get_prompt_system(),
            user_input=classification_prompt.get_user_prompt(),
            parameters_json=classification_prompt.get_parameters()
        )
        
        if not success:
            self.node.get_logger().error(f"There was a problem with classification prompt: {message}")
            return self.unknown_message(user_input, chat_history), COMMANDS.UNKNOWN, provider_used, model_used
        
        self.node.get_logger().info(f"LLM for Classification Prompt:\n{response}")
        classification_response_json = extract_json_from_code_block(response) # Gemini usually puts the response in ```json block
        if not classification_response_json:
            self.node.get_logger().error(f"No JSON format found.")
            return self.unknown_message(user_input, chat_history), COMMANDS.UNKNOWN, provider_used, model_used

        classification_response = try_json_loads(classification_response_json)
        if not classification_response:
            self.node.get_logger().error(f"Error on JSON loads.")
            return self.unknown_message(user_input, chat_history), COMMANDS.UNKNOWN, provider_used, model_used
        
        intent = classification_response["intent"]

        if intent == COMMANDS.DELETE_USER:
            user = classification_response["arguments"]["user"]
            if not user:
                response = f"No he eliminado a nadie, no sé a quién te refieres"
            else:
                users_obj_json = self.hri_engine.get_faceprint_request(json.dumps({"name": user}))
                users_obj = json.loads(users_obj_json)
                if len(users_obj) > 0:
                    user_obj = users_obj[0]
                    result = self.hri_engine.delete_request(user_obj["id"])
                    if result >= 0:
                        response = f"He eliminado a {user} correctamente"
                    else:
                        response = f"No he podido borrar a {user}"
                else:
                    response = f"No he encontrado a nadie con el nombre {user}"

        elif intent == COMMANDS.RENAME_USER:
            old_name = classification_response["arguments"]["old_name"]
            new_name = classification_response["arguments"]["new_name"]
            if not old_name:
                response = f"No he renombrado a nadie, no sé a quién te refieres"
            elif not new_name:
                response = f"No he renombrado a {old_name}, no sé que otro nombre le quieres poner"
            else:
                users_obj_json = self.hri_engine.get_faceprint_request(json.dumps({"name": old_name}))
                users_obj = json.loads(users_obj_json)
                if len(users_obj) > 0:
                    user_obj = users_obj[0]
                    result = self.hri_engine.rename_request(user_obj["id"], new_name)
                    if result >= 0:
                        response = f"Le he cambiado el nombre a {old_name} por {new_name} correctamente"
                    else:
                        response = f"No he podido cambiarle el nombre a {old_name} por {new_name}"
                else:
                    response = f"No he encontrado a nadie con el nombre {old_name}"

        elif intent == COMMANDS.TAKE_PICTURE:
            image = self.hri_engine.take_picture_request()
            if not image:
                response = "No he podido tomar una foto"
            else:
                response = "Foto tomada correctamente"

        else:
            response = self.unknown_message(user_input, chat_history)

        return response, intent, provider_used, model_used
