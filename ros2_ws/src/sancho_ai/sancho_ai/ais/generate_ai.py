import json

from .ai import AI
from ..llm_engine import LLMEngine
from ..prompts.commands.commands import COMMANDS

from ..prompts.unknown_prompt import UnknownPrompt
from ..prompts.semantic_response_prompt import SemanticResponsePrompt
from ..prompts.commands.commands import COMMANDS

"""
{
    "intent": "DELETE_USER",
    "arguments": {
        "user": ""
    },
    "output": {
        "status": "MISSING_ARGUMENT",
        "details": "No se ha podido ejecutar la acción.",
        "missing_arguments": ["user"]
    }
}
"""
def build_semantic_result(intent, arguments, status, details, missing_arguments):
    return {
        "intent": intent,
        "arguments": arguments,
        "output": {
            "status": status,
            "details": details,
            "missing_arguments": missing_arguments
        }
    }


class GenerateAI(AI):

    def __init__(self, llm_engine: LLMEngine):
        self.llm_engine = llm_engine

    def do_semantic_response_prompt(self, user_input, semantic_result, chat_history):
        semantic_response_prompt = SemanticResponsePrompt(user_input, semantic_result, chat_history)
        response, provider_used, model_used, message, success = self.llm_engine.prompt_request(
            messages_json=json.dumps(chat_history), # Ultimos 5 turnos
            prompt_system=semantic_response_prompt.get_prompt_system(),
            user_input=semantic_response_prompt.get_user_prompt(),
            parameters_json=semantic_response_prompt.get_parameters()
        )

        if not success:
            self.node.get_logger().error(f"There was a problem with semantic response prompt: {message}")
            return "Ha ocurrido un error, lo siento bro"
        
        self.node.get_logger().info(f"LLM for Semantic Response Prompt:\n{response}")

        return response
    
    def how_are_you(self, user_input, chat_history=[]):
        result = build_semantic_result(COMMANDS.HOW_ARE_YOU.value, {}, "SUCCESS", "Estoy funcionado correctamente y sin errores.", [])
        
        return self.do_semantic_response_prompt(user_input, result, chat_history)

    
    def what_you_see(self, actual_people, user_input, chat_history=[]):
        people_on_screen = [person for person, time_on_screen in actual_people.items() if time_on_screen < 1]
        if not people_on_screen:
            result = build_semantic_result(COMMANDS.WHAT_YOU_SEE.value, {}, "SUCCESS", "No veo a nadie.", [])
        elif len(people_on_screen) == 1:
            result = build_semantic_result(COMMANDS.WHAT_YOU_SEE.value, {}, "SUCCESS", f"Veo a {people_on_screen[0]}.", [])
        else:
            people_str = ", ".join(people_on_screen[:-1]) + f" y {people_on_screen[-1]}"
            result = build_semantic_result(COMMANDS.WHAT_YOU_SEE.value, {}, "SUCCESS", f"Veo a {people_str}.", [])
        
        return self.do_semantic_response_prompt(user_input, result, chat_history)

    def delete_user(self, user, result, user_input, chat_history=[]):
        if user:
            status = "SUCCESS" if result == "success" else "FAILURE"
            detail = f"Se ha eliminado a {user} correctamente" if result == "success" else f"No se pudo eliminar a {user}"
            result = build_semantic_result(COMMANDS.DELETE_USER.value, {"user": user}, status, detail, [])
        else:
            status = "MISSING_ARGUMENT"
            detail = "No se ha especificado el usuario a eliminar."
            result = build_semantic_result(COMMANDS.DELETE_USER.value, {"user": ""}, status, detail, ["user"])

        return self.do_semantic_response_prompt(user_input, result, chat_history)

    def unknown_message(self, user_input, chat_history=[]):
        unknown_prompt = UnknownPrompt(user_input)

        self.node.get_logger().info(f"User: {user_input}")
        response, provider_used, model_used, message, success = self.llm_engine.prompt_request(
            messages_json=json.dumps(chat_history), # Ultimos 5 turnos
            prompt_system=unknown_prompt.get_prompt_system(),
            user_input=unknown_prompt.get_user_prompt(),
            parameters_json=unknown_prompt.get_parameters()
        )
        
        if not success:
            self.node.get_logger().error(f"There was a problem with unknown prompt: {message}")
            return super().unknown_message()
        
        self.node.get_logger().info(f"LLM for Unknown Prompt:\n{response}")

        return response
