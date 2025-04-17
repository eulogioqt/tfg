import json
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline
from sentence_transformers import SentenceTransformer

from .llm_provider import LLMProvider
from ..constants import LLAMA_LLM_MODEL, LLAMA_EMBEDDING_MODEL
import sys
class LLaMAProvider(LLMProvider):

    def __init__(self, api_key):
        self.device = 0 if torch.cuda.is_available() else -1

        llm_model_name = LLAMA_LLM_MODEL.MISTRAL_7B
        tokenizer = AutoTokenizer.from_pretrained(llm_model_name.value, token=api_key)
        model = AutoModelForCausalLM.from_pretrained(
            llm_model_name.value,
            torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
            device_map="auto" if torch.cuda.is_available() else None, 
            token=api_key
        )

        self.client_embeddings = SentenceTransformer(LLAMA_EMBEDDING_MODEL.MINI_LM_L6_V2.value)
        self.client = {
            LLAMA_LLM_MODEL.MISTRAL_7B: pipeline("text-generation", model=model, tokenizer=tokenizer)
        }

    def embedding(self, model, user_input):
        if not model:
            model = LLAMA_EMBEDDING_MODEL.MINI_LM_L6_V2

        return self.client_embeddings.encode([user_input])[0].tolist()
    
    def prompt(self, node, model, prompt_system, messages_json, user_input, parameters_json):
        full_prompt = ""

        if not model:
            model = LLAMA_LLM_MODEL.MISTRAL_7B

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
        node.get_logger().info(full_prompt)
        parameters = json.loads(parameters_json) if parameters_json else {}

        final_parameters = {}
        final_parameters["temperature"] = parameters.get("temperature", 0.0)
        final_parameters["max_new_tokens"] = parameters.get("max_tokens", 60)

        response = self.client[model](full_prompt, **final_parameters)
        return response[0]["generated_text"][len(full_prompt):].strip()

