import gc
import json
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer

from .base_provider import BaseProvider
from ..prompt_formatters import HFChatTemplateFormatter


class HFTextGenerationProvider(BaseProvider):
    def __init__(self, models=None, api_key=None, model_formatters=None):
        self.api_key = api_key
        self.model_formatters = model_formatters
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.models = {}
        self.tokenizers = {}
        self.formatters = {}

        if models:
            self.load(models)

    def embedding(self, *args, **kwargs):
        raise NotImplementedError("This provider does not support embeddings.")

    def prompt(self, model, prompt_system, messages_json, user_input, parameters_json):
        model = model or self.models.values()[0]
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
            "do_sample": parameters.get("temperature", 0.0) > 0.0
        }

        generated_ids = model_obj.generate(model_inputs, **final_parameters)
        new_tokens = generated_ids[0][model_inputs.shape[1]:]
        response = tokenizer.decode(new_tokens, skip_special_tokens=True)

        return response

    def load(self, models):
        for model in models:
            tokenizer = AutoTokenizer.from_pretrained(model.value, token=self.api_key)
            model = AutoModelForCausalLM.from_pretrained(
                model.value,
                torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
                device_map="auto" if torch.cuda.is_available() else None,
                token=self.api_key
            )

            self.models[model] = model
            self.tokenizers[model] = tokenizer
            self.formatters[model] = (self.model_formatters or {}).get(model, HFChatTemplateFormatter)(tokenizer)

    def unload(self, models):
        if not models:
            models = list(self.models.keys())

        for model in models:
            del self.models[model]
            del self.tokenizers[model]
            del self.formatters[model]

        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
