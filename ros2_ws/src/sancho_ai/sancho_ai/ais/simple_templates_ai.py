"""TODO: Add module documentation."""
from ..ais.base import ModularAI
from ..ais.intent_classifiers import SimpleClassifier
from ..ais.intent_executors import IntentExecutor
from ..ais.response_generators import TemplatesGenerator

from ..engines import  HRIEngine


class SimpleTemplatesAI(ModularAI):

"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        node = HRIEngine.create_client_node()
        hri_engine = HRIEngine(node)

        simple_classifier = SimpleClassifier()
        intent_executor = IntentExecutor(hri_engine)
        templates_generator = TemplatesGenerator()

        super().__init__(simple_classifier, intent_executor, templates_generator)
