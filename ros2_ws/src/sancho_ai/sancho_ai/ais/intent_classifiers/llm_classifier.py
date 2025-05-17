from .intent_classifier import IntentClassifier

from ...engines import LLMEngine
from ...prompts.classification_prompt import ClassificationPrompt
from ...prompts.commands.commands import COMMANDS


class LLMClassifier(IntentClassifier):

    def __init__(self, llm_engine: LLMEngine):
        self.llm_engine = llm_engine
    
    def classify(self, user_input, chat_history=[]):
        classification_prompt = ClassificationPrompt(user_input, chat_history) # Ultimos 5 turnos

        print(f"User: {user_input}")
        response, provider_used, model_used, message, success = self.llm_engine.prompt_request(
            prompt_system=classification_prompt.get_prompt_system(),
            user_input=classification_prompt.get_user_prompt(),
            parameters_json=classification_prompt.get_parameters()
        )
        
        if not success:
            print(f"There was a problem with classification prompt: {message}")
            return COMMANDS.UNKNOWN, {}, provider_used, model_used
        
        print(f"LLM for Classification Prompt:\n{response}")
        classification_response_json = ClassificationPrompt.extract_json_from_code_block(response) # Gemini usually puts the response in ```json block
        if not classification_response_json:
            print(f"No JSON format found.")
            return COMMANDS.UNKNOWN, {}, provider_used, model_used

        classification_response = ClassificationPrompt.try_json_loads(classification_response_json)
        if not classification_response:
            print(f"Error on JSON loads.")
            return COMMANDS.UNKNOWN, {}, provider_used, model_used
        
        intent = classification_response["intent"]
        arguments = classification_response["arguments"]

        return intent, arguments, provider_used, model_used
