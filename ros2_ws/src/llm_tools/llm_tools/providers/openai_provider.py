import json
import openai

from .llm_provider import LLMProvider
from ..constants import OPENAI_EMBEDDING_MODEL, OPENAI_LLM_MODEL

class OpenAIProvider(LLMProvider):

    def __init__(self, api_key):
        self.client = openai.OpenAI(api_key=api_key)
    
    def embedding(self, model, user_input):
        if not model:
            model = OPENAI_EMBEDDING_MODEL.SMALL_3

        return self.client.embeddings.create(model=model, input=[user_input]).data[0].embedding

    def prompt(self, model, prompt_system, messages_json, user_input, parameters_json):       
        messages = []

        if not model: 
            model = OPENAI_LLM_MODEL.GPT_3_5_TURBO

        if prompt_system: 
            messages.append({"role": "system", "content": prompt_system})

        if messages_json:
            messages.extend(json.loads(messages_json)) 

        if user_input:
            messages.append({"role": "user", "content": user_input})

        parameters = json.loads(parameters_json) if parameters_json else {}

        final_parameters = {}
        final_parameters["temperature"] = parameters.get("temperature", 0.0)
        final_parameters["max_tokens"] = parameters.get("max_tokens", 60)

        response = self.client.chat.completions.create(model=model, messages=messages, **final_parameters)
        return response.choices[0].message.content
