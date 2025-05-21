import json
from datetime import datetime

from .response_generator import ResponseGenerator

from ...log_manager import LogManager
from ...prompts import UnknownPrompt, SemanticResultPrompt
from ...engines import LLMEngine, HRIEngine


class LLMGenerator(ResponseGenerator):

    def __init__(self, hri_engine: HRIEngine, llm_engine: LLMEngine):
        self.hri_engine = hri_engine
        self.llm_engine = llm_engine
    
    def generate_response(self, details: str, status: str, intent: str, arguments: dict, user_input: str, chat_history: list) -> str:
        semantic_result = SemanticResultPrompt.build_semantic_result(intent, arguments, status, details)
        semantic_response_prompt = SemanticResultPrompt(semantic_result, user_input, chat_history)
        LogManager.info(f"PROMPT SYSTEM:\n{semantic_response_prompt.get_prompt_system()}")
        
        response, provider_used, model_used, message, success = self.llm_engine.prompt_request(
            prompt_system=semantic_response_prompt.get_prompt_system(),
            user_input=semantic_response_prompt.get_user_prompt(),
            parameters_json=semantic_response_prompt.get_parameters()
        )

        if not success:
            LogManager.error(f"There was a problem with generation prompt: {message}")
            return semantic_result["details"]
        
        LogManager.info(f"LLM for Generation Prompt:\n{response}")
        
        emotion = "neutral"

        return response, emotion, provider_used, model_used

    def continue_conversation(self, user_input: str, chat_history: list) -> str:
        robot_context = self._build_robot_context()

        unknown_prompt = UnknownPrompt(user_input, robot_context)
        LogManager.info(f"Prompt system: {unknown_prompt.get_prompt_system()}")
        LogManager.info(f"User: {user_input}")

        response, provider_used, model_used, message, success = self.llm_engine.prompt_request(
            messages_json=json.dumps(chat_history),
            prompt_system=unknown_prompt.get_prompt_system(),
            user_input=unknown_prompt.get_user_prompt(),
            parameters_json=unknown_prompt.get_parameters()
        )

        if not success:
            LogManager.error(f"There was a problem with unknown prompt: {message}")
            return "Lo siento, no te he entendido", "sad", provider_used, model_used
        
        LogManager.info(f"LLM for Classification Prompt:\n{response}")
        classification_response_json = SemanticResultPrompt.extract_json_from_code_block(response) # Gemini usually puts the response in ```json block
        if not classification_response_json:
            LogManager.error(f"No JSON format found.")
            return "Lo siento, no te he entendido", "sad", provider_used, model_used

        classification_response = SemanticResultPrompt.try_json_loads(classification_response_json)
        if not classification_response:
            LogManager.error(f"Error on JSON loads.")
            return "Lo siento, no te he entendido", "sad", provider_used, model_used
        
        text_response = classification_response.get("response", "")
        emotion = classification_response.get("emotion", "")
        if not text_response:
            LogManager.error(f"The JSON response does not contain a text response.")
            return "Lo siento, no te he entendido", "sad", provider_used, model_used
        
        if not emotion or emotion not in ["happy", "surprised", "sad", "angry", "bored", "suspicious", "neutral"]:
            LogManager.error(f"❌❌❌❌❌ EMOTION {emotion} INVALID. Default to neutral.")
            emotion = "neutral"

        LogManager.info(f"LLM for Unknown Prompt:\n{text_response}\nEmotion:\n{emotion}")

        return text_response, emotion, provider_used, model_used
    
    def _build_robot_context(self):
        actual_people_json = self.hri_engine.get_actual_people_request()
        actual_people = json.loads(actual_people_json)
        visible_ids = [int(fpid) for fpid, t in actual_people.items() if t < 1]

        faceprints_json = self.hri_engine.get_faceprint_request(json.dumps({"fields": ["id", "name"]}))
        faceprints = json.loads(faceprints_json)
        id_to_name = {int(fp["id"]): fp["name"] for fp in faceprints}

        visible_people = [id_to_name[pid] for pid in visible_ids if pid in id_to_name]
        known_people = list(id_to_name.values())

        sessions_summary_json = self.hri_engine.get_sessions_summary_request()
        sessions_summary = json.loads(sessions_summary_json)

        times_seen = {}
        last_seen = {}
        for summary in sessions_summary:
            fp_id = int(summary["faceprint_id"])
            name = id_to_name.get(fp_id)
            if not name: # Esta persona ya no existe (habria q hacer q borrar un faceprint lo borre de sessions)
                continue
            
            dt = datetime.fromtimestamp(float(summary["last_seen"]))
            last_seen[name] = dt.strftime("%Y-%m-%d %H:%M")
            times_seen[name] = summary["sessions_count"]

        return {
            "visible_people": visible_people,
            "known_people": known_people,
            "times_seen": times_seen,
            "last_seen": last_seen
        }
