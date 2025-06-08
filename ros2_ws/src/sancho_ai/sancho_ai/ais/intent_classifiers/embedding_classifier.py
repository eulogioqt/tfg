from .intent_classifier import IntentClassifier

from ...log_manager import LogManager
from ...engines import EmbeddingEngine
from ...prompts.commands.commands import COMMANDS

import json
import os
import numpy as np


class EmbeddingClassifier(IntentClassifier):

    SIMILARITY_THRESHOLD = 0.8

    def __init__(self, embedding_engine: EmbeddingEngine, examples_path: str, provider: str, model: str):
        self.embedding_engine = embedding_engine
        self.examples_path = examples_path
        self.provider = provider
        self.model = model

        with open(self.examples_path, 'r') as f:
            self.examples = json.load(f)

    def classify(self, user_input, _):
        LogManager.info(f"User: {user_input}")

        user_embedding, provider_used, model_used, message, success = self.embedding_engine.embedding_request(
            provider=self.provider,
            model=self.model,
            user_input=user_input
        )

        if not success:
            LogManager.error(f"Error in embedding request: {message}")
            return COMMANDS.UNKNOWN, {}, provider_used, model_used

        user_embedding = np.array(user_embedding)

        best_intent = COMMANDS.UNKNOWN
        best_similarity = -1.0

        for intent, examples in self.examples.items():
            for phrase, models in examples.items():
                if self.model in models:
                    example_embedding = np.array(models[self.model])
                    similarity = self.cosine_similarity(user_embedding, example_embedding)
                    if similarity > best_similarity:
                        best_similarity = similarity
                        best_intent = intent

        if best_similarity >= self.SIMILARITY_THRESHOLD and best_intent in list(COMMANDS):
            return best_intent, {}, provider_used, model_used
        else:
            LogManager.info(f"Embedding similarity too low ({best_similarity:.2f}), classified as UNKNOWN")
            return COMMANDS.UNKNOWN, {}, provider_used, model_used

    @staticmethod
    def cosine_similarity(a, b):
        return np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9)
