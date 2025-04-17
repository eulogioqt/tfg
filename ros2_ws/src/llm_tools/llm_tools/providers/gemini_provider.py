import json
import google.generativeai as genai

from .base_provider import BaseProvider
from ..prompt_formatters import GeminiFormatter

from ..constants import MODELS


class GeminiProvider(BaseProvider):
    def __init__(self, api_key: str):
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

        formatted = self.formatter.format(prompt_system, messages_json, user_input)
        chat = self.client[model].start_chat(history=formatted["history"])

        parameters = json.loads(parameters_json) if parameters_json else {}
        temperature = parameters.get("temperature", 0.0)

        response = chat.send_message(formatted["history"][-1]["parts"][0], generation_config={
            "temperature": temperature
        })

        return response.text.strip()
