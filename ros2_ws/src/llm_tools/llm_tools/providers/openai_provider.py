import json
import openai

from .llm_provider import LLMProvider

class OpenAIProvider(LLMProvider):

    def __init__(self, api_key):
        self.client = openai.OpenAI(api_key=api_key)
    
    def embedding(self, model, user_input):
        if not model:
            model = "text-embedding-3-small"

        return self.client.embeddings.create(
            model=model,
            input=[user_input]
        ).data[0].embedding

    def prompt(self, model, prompt_system, messages_json, user_input, parameters_json):       
        messages = []

        if not model: 
            model = "gpt-3.5-turbo"

        if prompt_system: 
            messages.append({"role": "system", "content": prompt_system})

        if messages_json:
            messages.extend(json.loads(messages_json)) 

        if user_input:
            messages.append({"role": "user", "content": user_input})

        parameters = json.loads(parameters_json) if parameters_json else {}
        parameters.setdefault("temperature", 0.0)
        parameters.setdefault("max_tokens", 60)

        response = self.client.chat.completions.create(
            model=model,
            messages=messages,
            **parameters
        )

        return response.choices[0].message.content
