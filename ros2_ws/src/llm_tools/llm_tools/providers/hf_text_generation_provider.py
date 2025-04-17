import torch
import json
from transformers import AutoModelForCausalLM, AutoTokenizer

from .base_provider import BaseProvider
from llm_tools.prompt_formatters import HFFormatter


class HFTextGenerationProvider(BaseProvider):
    def __init__(self, model_enum_list, api_key):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.models = {}
        self.tokenizers = {}
        self.formatters = {}
        self.default_model = model_enum_list[1]

        for model_enum in model_enum_list:
            tokenizer = AutoTokenizer.from_pretrained(model_enum.value, token=api_key)
            model = AutoModelForCausalLM.from_pretrained(
                model_enum.value,
                torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
                device_map="auto" if torch.cuda.is_available() else None,
                token=api_key
            )

            self.models[model_enum] = model
            self.tokenizers[model_enum] = tokenizer
            self.formatters[model_enum] = HFFormatter(tokenizer)

        test_response = self.prompt(
            model="",
            prompt_system="Esto es un pedazo de prompt system",
            messages_json="",
            user_input="Qué pasa picha, soy Joaquín",
            parameters_json=json.dumps({"temperature": 1.0, "max_tokens": 60})
        )
        print(test_response)

    def embedding(self, *args, **kwargs):
        raise NotImplementedError("This provider does not support embeddings.")

    def prompt(self, model, prompt_system, messages_json, user_input, parameters_json):
        model = model or self.default_model
        if model not in self.models:
            raise ValueError(f"Model {model} not loaded in provider.")

        tokenizer = self.tokenizers[model]
        formatter = self.formatters[model]
        model_obj = self.models[model]

        model_inputs = formatter.format(prompt_system, messages_json, user_input).to(self.device)
        parameters = json.loads(parameters_json) if parameters_json else {}
        final_parameters = {
            "temperature": parameters.get("temperature", 0.0),
            "max_new_tokens": parameters.get("max_tokens", 60),
            "do_sample": final_parameters["temperature"] > 0.0
        }

        generated_ids = model_obj.generate(model_inputs, **final_parameters)
        new_tokens = generated_ids[0][model_inputs.shape[1]:]
        response = tokenizer.decode(new_tokens, skip_special_tokens=True)

        return response
