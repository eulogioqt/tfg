import torch
import json
from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline

from .base_provider import BaseProvider
from llm_tools.prompt_formatters import HFFormatter


class HFTextGenerationProvider(BaseProvider):
    def __init__(self, model_enum_list, api_key):
        self.device = 0 if torch.cuda.is_available() else -1

        self.client = {}
        self.formatters = {}
        self.default_model = model_enum_list[0]

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
            self.formatters[model_enum] = HFFormatter(tokenizer)

        a = self.prompt("", "esto es un pedazo de prompt system", "", "que pasa picha soy joaquin", json.dumps({"temperature": 1.0, "max_tokens": 60}))
        print(a)

    def embedding(self, *args, **kwargs):
        raise NotImplementedError("This provider does not support embeddings.")

    def prompt(self, model, prompt_system, messages_json, user_input, parameters_json):
        model = model or self.default_model

        if model not in self.client:
            raise ValueError(f"Model {model} not loaded in provider.")

        formatted_prompt = self.formatters[model].format(prompt_system, messages_json, user_input)

        print("\n🧾 Formatted Prompt:\n" + "-" * 40 + f"\n{formatted_prompt}\n" + "-" * 40)

        parameters = json.loads(parameters_json) if parameters_json else {}
        final_parameters = {
            "temperature": parameters.get("temperature", 0.0),
            "max_new_tokens": parameters.get("max_tokens", 60)
        }

        response = self.client[model](formatted_prompt, **final_parameters)
        return response[0]["generated_text"][len(formatted_prompt):].strip()
