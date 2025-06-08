import os
import json
import time
from pathlib import Path
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from dotenv import load_dotenv

from llm_msgs.srv import  LoadModel, UnloadModel
from llm_msgs.msg import LoadModel as LoadModelMsg, ProviderModel
from llm_tools.models import PROVIDER, MODELS, NEEDS_API_KEY

from .ais.intent_classifiers import LLMClassifier
from .engines import LLMEngine
from .prompts.commands.commands import COMMANDS


def compute_metrics_for_model(classifier: LLMClassifier, tests: list) -> Dict[str, Any]:
    metrics = {
        "total_tests": len(tests),
        "valid_json": 0,
        "invalid_json": 0,
        "intent_correct": 0,
        "intent_and_args_correct": 0,
        "intent_missing": 0,
        "intent_wrong": 0,
        "unknown_misclassified": 0,
        "made_up_intent": 0,
        "intent_correct_args_wrong": 0,
        "empty_response": 0,
        "total_time_sec": 0.0
    }

    for test in tests:
        chat_history = test["chat_history"][:-1]
        user_input = test["chat_history"][-1]["content"]
        expected = test["expected_output"]

        start = time.time()
        intent, arguments, _, _ = classifier.classify(user_input, chat_history)
        duration = time.time() - start
        metrics["total_time_sec"] += duration

        if intent == COMMANDS.UNKNOWN and expected["intent"] != COMMANDS.UNKNOWN:
            metrics["unknown_misclassified"] += 1
        elif intent == expected["intent"]:
            metrics["intent_correct"] += 1
            if arguments == expected.get("arguments", {}):
                metrics["intent_and_args_correct"] += 1
            else:
                metrics["intent_correct_args_wrong"] += 1
        else:
            metrics["intent_wrong"] += 1
            if intent not in list(COMMANDS):
                metrics["made_up_intent"] += 1

    return metrics


class TestLLMClassificationNode(Node):

    def __init__(self):
        super().__init__('intent_eval_node')
        load_dotenv()

        self.cli_load = self.create_client(LoadModel, 'llm_tools/load_model')
        self.cli_unload = self.create_client(UnloadModel, 'llm_tools/unload_model')

        self.base_dir = Path(__file__).parent
        self.tests_file = self.base_dir / 'prompts/commands/tests_dataset.json'
        self.results_file = self.base_dir / 'prompts/commands/results_llm.json'

        self.results = self.load_results()
        self.node_for_engine = self

    def load_results(self):
        if self.results_file.exists():
            with open(self.results_file, 'r') as f:
                return json.load(f)
        return {}

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
            if not model_enum:
                continue

            if provider.value not in self.results:
                self.results[provider.value] = {}

            for model in model_enum:
                model_name = str(model)
                if model_name in self.results[provider.value]:
                    self.get_logger().info(f"⏭️  Saltando {provider}/{model_name} (ya evaluado)")
                    continue

                self.get_logger().info(f"🚀 Evaluando {provider}/{model_name}")
                if not self.load_model(provider, model_name):
                    self.results[provider.value][model_name] = {"error": "No se pudo cargar el modelo"}
                    self.save_results()
                    continue

                engine = LLMEngine(self)
                classifier = LLMClassifier(engine)
                metrics = compute_metrics_for_model(classifier, tests)
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
    node = TestLLMClassificationNode()
    node.run_all_tests()
    node.destroy_node()
    rclpy.shutdown()
