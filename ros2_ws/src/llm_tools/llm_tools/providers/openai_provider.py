import json
import openai

from .base_provider import BaseProvider
from ..prompt_formatters import OpenAIFormatter
from ..models import MODELS


class OpenAIProvider(BaseProvider):

    def __init__(self, api_key):
        self.formatter = OpenAIFormatter()
        self.client = openai.OpenAI(api_key=api_key)
    
    def embedding(self, model, user_input):
        if not model:
            model = MODELS.EMBEDDING.OPENAI.SMALL_3

        return self.client.embeddings.create(model=model, input=[user_input]).data[0].embedding

    def prompt(self, model, prompt_system, messages_json, user_input, parameters_json):       
        if not model: 
            model = MODELS.LLM.OPENAI.GPT_3_5_TURBO

        messages = self.formatter.format(prompt_system, messages_json, user_input)
        parameters = json.loads(parameters_json) if parameters_json else {}
        final_parameters = {
            "temperature": parameters.get("temperature", 0.0),
            "max_tokens": parameters.get("max_tokens", 60)
        }

        response = self.client.chat.completions.create(model=model, messages=messages, **final_parameters)
        return response.choices[0].message.content
