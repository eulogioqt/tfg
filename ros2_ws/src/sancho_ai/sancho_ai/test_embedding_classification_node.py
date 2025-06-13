"""TODO: Add module documentation."""
import os 
import json
import time
from pathlib import Path
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from dotenv import load_dotenv

from llm_msgs.srv import LoadModel, UnloadModel
from llm_msgs.msg import LoadModel as LoadModelMsg, ProviderModel
from llm_tools.models import PROVIDER, MODELS, NEEDS_API_KEY

from .ais.intent_classifiers import EmbeddingClassifier
from .engines import EmbeddingEngine
from .prompts.commands.commands import COMMANDS

def compute_embedding_metrics(classifier: EmbeddingClassifier, tests: list, logger) -> Dict[str, Any]:
"""TODO: Describe compute_embedding_metrics.
Args:
    classifier (:obj:`Any`): TODO.
    tests (:obj:`Any`): TODO.
    logger (:obj:`Any`): TODO.
"""
    metrics = {
        "total_tests": len(tests),
        "intent_correct": 0,
        "intent_wrong": 0,
        "unknown_misclassified": 0,
        "made_up_intent": 0,
        "total_time_sec": 0.0
    }

    for idx, test in enumerate(tests):
        chat_history = test["chat_history"][:-1]
        user_input = test["chat_history"][-1]["content"]
        expected = test["expected_output"]

        logger.info(f"🧪 Test {idx+1}/{len(tests)}: '{user_input}'")

        start = time.time()
        intent, _, _, _ = classifier.classify(user_input, chat_history)
        duration = time.time() - start
        metrics["total_time_sec"] += duration

        logger.info(f"    🔍 Predicted: {intent} | Expected: {expected['intent']} | Time: {duration:.3f}s")

        if intent == COMMANDS.UNKNOWN and expected["intent"] != COMMANDS.UNKNOWN:
            metrics["unknown_misclassified"] += 1
        elif intent == expected["intent"]:
            metrics["intent_correct"] += 1
        else:
            metrics["intent_wrong"] += 1
            if intent not in list(COMMANDS):
                metrics["made_up_intent"] += 1

    return metrics


class TestEmbeddingClassificationNode(Node):

"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        super().__init__('intent_eval_embedding_node')
        load_dotenv()

        self.cli_load = self.create_client(LoadModel, 'llm_tools/load_model')
        self.cli_unload = self.create_client(UnloadModel, 'llm_tools/unload_model')
        self.service_node = EmbeddingEngine.create_client_node()

        self.base_dir = Path(__file__).parent
        self.tests_file = self.base_dir / 'prompts/tests/tests_classification_dataset.json'
        self.results_file = self.base_dir / 'prompts/tests/results_embeddings.json'

        self.results = self.load_results()

    def load_results(self):
    """TODO: Describe load_results.
"""
        if self.results_file.exists():
            with open(self.results_file, 'r') as f:
                return json.load(f)
        return {}

    def save_results(self):
    """TODO: Describe save_results.
"""
        with open(self.results_file, 'w') as f:
            json.dump(self.results, f, indent=2)

    def wait_for_service(self, client):
    """TODO: Describe wait_for_service.
Args:
    client (:obj:`Any`): TODO.
"""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service {client.srv_name} not available.")
            return False
        return True

    def call_sync(self, client, request):
    """TODO: Describe call_sync.
Args:
    client (:obj:`Any`): TODO.
    request (:obj:`Any`): TODO.
"""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def get_api_key_for(self, provider):
    """TODO: Describe get_api_key_for.
Args:
    provider (:obj:`Any`): TODO.
"""
        if provider not in NEEDS_API_KEY:
            return ""
        return os.environ.get(f"{provider.name.upper()}_API_KEY", "")

    def run_all_tests(self):
    """TODO: Describe run_all_tests.
"""
        with open(self.tests_file) as f:
            tests = json.load(f)

        for provider in PROVIDER:
            model_enum = getattr(MODELS.EMBEDDING, provider.name.upper(), None)
            if not model_enum:
                continue

            if provider.value not in self.results:
                self.results[provider.value] = {}

            for model in model_enum:
                model_name = str(model)
                if model_name in self.results[provider.value]:
                    self.get_logger().info(f"⏭️ Saltando {provider}/{model_name} (ya evaluado)")
                    continue

                self.get_logger().info(f"🚀 Evaluando {provider}/{model_name}")

                if not self.load_model(provider, model_name):
                    self.get_logger().error(f"❌ No se pudo cargar el modelo {provider}/{model_name}")
                    self.results[provider.value][model_name] = {"error": "No se pudo cargar el modelo"}
                    self.save_results()
                    continue

                engine = EmbeddingEngine(self.service_node)
                classifier = EmbeddingClassifier(
                    embedding_engine=engine,
                    provider=provider.value,
                    model=model_name
                )
                metrics = compute_embedding_metrics(classifier, tests, self.get_logger())
                self.results[provider.value][model_name] = metrics
                self.save_results()
                self.unload_model(provider, model_name)

        self.get_logger().info("✅ Evaluación embeddings finalizada.")

    def load_model(self, provider, model_name):
    """TODO: Describe load_model.
Args:
    provider (:obj:`Any`): TODO.
    model_name (:obj:`Any`): TODO.
"""
        if not self.wait_for_service(self.cli_load):
            return False
        req = LoadModel.Request()
        item = LoadModelMsg(provider=provider.value, models=[model_name], api_key=self.get_api_key_for(provider))
        req.items = [item]
        res = self.call_sync(self.cli_load, req)
        return res.results[0].success if res and res.results else False

    def unload_model(self, provider, model_name):
    """TODO: Describe unload_model.
Args:
    provider (:obj:`Any`): TODO.
    model_name (:obj:`Any`): TODO.
"""
        if not self.wait_for_service(self.cli_unload):
            return False
        req = UnloadModel.Request()
        item = ProviderModel(provider=provider.value, models=[model_name])
        req.items = [item]
        self.call_sync(self.cli_unload, req)


def main(args=None):
"""TODO: Describe main.
Args:
    args (:obj:`Any`): TODO.
"""
    rclpy.init(args=args)

    node = TestEmbeddingClassificationNode()
    node.run_all_tests()
    node.destroy_node()

    rclpy.shutdown()
