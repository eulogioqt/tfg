import os
import json
from pathlib import Path
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from dotenv import load_dotenv

from llm_msgs.srv import Prompt, LoadModel, UnloadModel
from llm_msgs.msg import LoadModel as LoadModelMsg, ProviderModel
from llm_tools.models import PROVIDER, MODELS, NEEDS_API_KEY

# mejorar este nodo que tambien saque metricas de tiempo y que no use magic strings

def safe_json_parse(text: str) -> Any:
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return None

def compute_metrics_for_model(node, provider: str, model: str, tests: list) -> Dict[str, Any]:
    total = len(tests)

    results = {
        "total_tests": total,
        "valid_json": 0,
        "invalid_json": 0,
        "intent_correct": 0,
        "intent_and_args_correct": 0,
        "intent_missing": 0,
        "intent_wrong": 0,
        "unknown_misclassified": 0,
        "made_up_intent": 0,
        "intent_correct_args_wrong": 0,
        "empty_response": 0
    }

    for i, test in enumerate(tests):
        req = node.cli_prompt.srv_type.Request()
        req.provider = provider
        req.model = model
        req.prompt_system = "You are a helpful assistant."
        req.messages_json = json.dumps(test["chat_history"])
        req.user_input = ""
        req.parameters_json = json.dumps({"temperature": 0.0, "max_tokens": 200})

        res = node.call_sync(node.cli_prompt, req)

        if not res or not res.success:
            results["empty_response"] += 1
            continue

        output = safe_json_parse(res.response)
        if output is None:
            results["invalid_json"] += 1
            continue

        results["valid_json"] += 1
        expected = test["expected_output"]

        if "intent" not in output:
            results["intent_missing"] += 1
            continue

        predicted_intent = output["intent"]
        expected_intent = expected["intent"]

        if predicted_intent == expected_intent:
            results["intent_correct"] += 1

            # Comparar argumentos exactos
            expected_args = expected.get("arguments", {})
            predicted_args = output.get("arguments", {})
            if predicted_args == expected_args:
                results["intent_and_args_correct"] += 1
            else:
                results["intent_correct_args_wrong"] += 1
        else:
            results["intent_wrong"] += 1

            # ¿Era UNKNOWN y falló?
            if expected_intent == "UNKNOWN":
                results["unknown_misclassified"] += 1

            # ¿Intención inventada?
            allowed = {"DELETE_USER", "RENAME_USER", "TAKE_PICTURE", "UNKNOWN"}
            if predicted_intent not in allowed:
                results["made_up_intent"] += 1

    return results


class TestClassificationNode(Node):

    def __init__(self):
        super().__init__('intent_eval_node')
        load_dotenv()

        self.cli_prompt = self.create_client(Prompt, 'llm_tools/prompt')
        self.cli_load = self.create_client(LoadModel, 'llm_tools/load_model')
        self.cli_unload = self.create_client(UnloadModel, 'llm_tools/unload_model')

        self.base_dir = Path(__file__).parent
        self.results_file = self.base_dir / 'results.json'
        self.tests_file = self.base_dir / 'prompts/commands/tests.json'
        self.load_results()

    def load_results(self):
        if self.results_file.exists():
            with open(self.results_file, 'r') as f:
                self.results = json.load(f)
        else:
            self.results = {}

    def save_results(self):
        with open(self.results_file, 'w') as f:
            json.dump(self.results, f, indent=2)

    def wait_for_service(self, client):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service {client.srv_name} not available.")
            return False
        return True

    def call_sync(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def get_api_key_for(self, provider):
        if provider not in NEEDS_API_KEY:
            return ""
        return os.environ.get(f"{provider.name.upper()}_API_KEY", "")

    def run_all_tests(self):
        with open(self.tests_file) as f:
            tests = json.load(f)

        for provider in PROVIDER:
            model_enum = getattr(MODELS.LLM, provider.name.upper(), None)
            if model_enum is None:
                continue

            if provider.value not in self.results:
                self.results[provider.value] = {}

            for model in model_enum:
                model_name = str(model)
                if model_name in self.results[provider.value]:
                    self.get_logger().info(f"⏭️  Saltando {provider}/{model_name} (ya evaluado)")
                    continue

                self.get_logger().info(f"🚀 Evaluando {provider}/{model_name}")
                success = self.load_model(provider, model_name)
                if not success:
                    self.results[provider.value][model_name] = {"error": "No se pudo cargar el modelo"}
                    self.save_results()
                    continue

                metrics = compute_metrics_for_model(self, provider.value, model_name, tests)
                self.results[provider.value][model_name] = metrics
                self.save_results()
                self.unload_model(provider, model_name)

        self.get_logger().info("✅ Evaluación finalizada.")

    def load_model(self, provider, model_name):
        if not self.wait_for_service(self.cli_load):
            return False
        req = LoadModel.Request()
        item = LoadModelMsg(provider=provider.value, models=[model_name], api_key=self.get_api_key_for(provider))
        req.items = [item]
        res = self.call_sync(self.cli_load, req)
        return res.results[0].success if res and res.results else False

    def unload_model(self, provider, model_name):
        if not self.wait_for_service(self.cli_unload):
            return
        req = UnloadModel.Request()
        item = ProviderModel(provider=provider.value, models=[model_name])
        req.items = [item]
        self.call_sync(self.cli_unload, req)


def main(args=None):
    rclpy.init(args=args)
    node = TestClassificationNode()
    node.run_all_tests()
    node.destroy_node()
    rclpy.shutdown()
