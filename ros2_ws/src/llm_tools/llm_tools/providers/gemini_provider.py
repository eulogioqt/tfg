import json
import google.generativeai as genai

from .base_provider import BaseProvider
from ..prompt_formatters import GeminiFormatter

from ..models import MODELS


class GeminiProvider(BaseProvider):
    def __init__(self, api_key):
        genai.configure(api_key=api_key)

        self.formatter = GeminiFormatter()
        self.client = {
            MODELS.LLM.GEMINI.GEMINI_FLASH: genai.GenerativeModel(MODELS.LLM.GEMINI.GEMINI_FLASH.value)
        }

    def embedding(self, *args, **kwargs):
        raise NotImplementedError("This provider does not support embeddings.")

    def prompt(self, model, prompt_system, messages_json, user_input, parameters_json):
        if not model:
            model = MODELS.LLM.GEMINI.GEMINI_FLASH

        messages = self.formatter.format(prompt_system, messages_json, user_input)
        chat = self.client[model].start_chat(history=messages)

        parameters = json.loads(parameters_json) if parameters_json else {}
        final_parameters = {
            "temperature": parameters.get("temperature", 0.0),
            "max_output_tokens": parameters.get("max_tokens", 60)
        }

        response = chat.send_message(messages[-1]["parts"][0], generation_config=final_parameters)

        return response.text.strip()
