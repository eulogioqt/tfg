"""TODO: Add module documentation."""
from .base import ModularAI
from .intent_classifiers import LLMClassifier
from .intent_executors import IntentExecutor
from .response_generators import TemplatesGenerator

from ..engines import HRIEngine, LLMEngine


class LLMClassifierTemplatesAI(ModularAI):

"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        node = HRIEngine.create_client_node()
        hri_engine = HRIEngine(node)
        llm_engine = LLMEngine(node)

        llm_classifier = LLMClassifier(llm_engine)
        intent_executor = IntentExecutor(hri_engine)
        templates_generator = TemplatesGenerator()

        super().__init__(llm_classifier, intent_executor, templates_generator)
