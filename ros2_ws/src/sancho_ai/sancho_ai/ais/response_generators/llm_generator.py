import json

from .response_generator import ResponseGenerator

from ...prompts import UnknownPrompt, SemanticResultPrompt
from ...engines import LLMEngine, HRIEngine


class LLMGenerator(ResponseGenerator):

    def __init__(self, hri_engine: HRIEngine, llm_engine: LLMEngine):
        self.hri_engine = hri_engine
        self.llm_engine = llm_engine
    
    def generate_response(self, details: str, status: str, intent: str, 
                          arguments: dict, user_input: str, chat_history: list) -> str:
        semantic_result = SemanticResultPrompt.build_semantic_result(intent, arguments, status, details)
        semantic_response_prompt = SemanticResultPrompt(semantic_result, user_input, chat_history)
        print(f"PROMPT SYSTEM:\n{semantic_response_prompt.get_prompt_system()}")
        
        response, provider_used, model_used, message, success = self.llm_engine.prompt_request(
            prompt_system=semantic_response_prompt.get_prompt_system(),
            user_input=semantic_response_prompt.get_user_prompt(),
            parameters_json=semantic_response_prompt.get_parameters()
        )

        if not success:
            print(f"There was a problem with generation prompt: {message}")
            return semantic_result["details"]
        
        print(f"LLM for Generation Prompt:\n{response}")

        return response, provider_used, model_used

    def continue_conversation(self, user_input: str, chat_history: list) -> str:
        import time
        a = time.time()
        actual_people_json = self.hri_engine.get_actual_people_request()
        actual_people = json.loads(actual_people_json)
        visible_people = [person for person, time_on_screen in actual_people.items() if time_on_screen < 1]

        faceprints_json = self.hri_engine.get_faceprint_request(json.dumps({"fields": ["id", "name"]}))
        faceprints = json.loads(faceprints_json)
        known_people = [fp["name"] for fp in faceprints]

        sessions_json = self.hri_engine.get_sessions_request()
        sessions = json.loads(sessions_json)
        times_seen = {}
        last_seen = {}

        robot_context = {
            "visible_people": visible_people,
            "known_people": known_people,
            "times_seen": {
                "Lucía": 5,
                "Pedro": 2,
                "Ana": 7
            },
            "last_seen": {
                "Lucía": "2025-05-16 10:22",
                "Pedro": "2025-05-15 17:50",
                "Ana": "2025-05-14 09:15"
            }
        }

        robot_context = {
            "visible_people": visible_people,
            "known_people": known_people,
            "times_seen": times_seen,
            "last_seen": last_seen
        }

        b = time.time() - a
        print(f"Time building context: {b}")

        unknown_prompt = UnknownPrompt(user_input, robot_context)
        print(f"Prompt system: {unknown_prompt.get_prompt_system()}")
        print(f"User: {user_input}")
        response, provider_used, model_used, message, success = self.llm_engine.prompt_request(
            messages_json=json.dumps(chat_history), # Ultimos 5 turnos
            prompt_system=unknown_prompt.get_prompt_system(),
            user_input=unknown_prompt.get_user_prompt(),
            parameters_json=unknown_prompt.get_parameters()
        )
        
        if not success:
            print(f"There was a problem with unknown prompt: {message}")
            return "Lo siento, no te he entendido"
        
        print(f"LLM for Unknown Prompt:\n{response}")

        return response, provider_used, model_used