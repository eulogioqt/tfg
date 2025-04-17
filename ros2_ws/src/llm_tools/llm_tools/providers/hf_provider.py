import torch
import json
from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline

from .llm_provider import LLMProvider


class HFProvider(LLMProvider):
    def __init__(self, model_enum_list, api_key):
        self.device = 0 if torch.cuda.is_available() else -1
        self.client = {}

        for model_enum in model_enum_list:
            tokenizer = AutoTokenizer.from_pretrained(model_enum.value, token=api_key)
            model = AutoModelForCausalLM.from_pretrained(
                model_enum.value,
                torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
                device_map="auto" if torch.cuda.is_available() else None,
                token=api_key
            )
            self.client[model_enum] = pipeline(
                "text-generation",
                model=model,
                tokenizer=tokenizer
            )

        self.default_model = model_enum_list[0]

    def embedding(self, model, user_input):
        raise NotImplementedError()

    def format_prompt(self, prompt_system, messages_json, user_input):
        full_prompt = ""

        if prompt_system:
            full_prompt += f"[INST] <<SYS>>\n{prompt_system}\n<</SYS>>\n"

        if messages_json:
            for msg in json.loads(messages_json):
                if msg["role"] == "user":
                    full_prompt += f"[INST] {msg['content']} [/INST]\n"
                elif msg["role"] == "assistant":
                    full_prompt += f"{msg['content']}\n"

        if user_input:
            full_prompt += f"\nInput: {user_input}"

        full_prompt += "\n[/INST]"
        return full_prompt

    def prompt(self, model, prompt_system, messages_json, user_input, parameters_json):
        model = model or self.default_model
        if model not in self.client:
            raise ValueError(f"Model {model} not loaded in provider.")

        full_prompt = self.format_prompt(prompt_system, messages_json, user_input)

        parameters = json.loads(parameters_json) if parameters_json else {}
        final_parameters = {
            "temperature": parameters.get("temperature", 0.0),
            "max_new_tokens": parameters.get("max_tokens", 60)
        }

        response = self.client[model](full_prompt, **final_parameters)
        return response[0]["generated_text"][len(full_prompt):].strip()
