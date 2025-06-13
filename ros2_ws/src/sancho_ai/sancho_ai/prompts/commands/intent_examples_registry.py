"""TODO: Add module documentation."""
import os
import json

class IntentExamplesRegistry:
"""TODO: Describe class."""
    _intents = None

    @classmethod
    def get_intent_examples(cls):
    """TODO: Describe get_intent_examples.
Args:
    cls (:obj:`Any`): TODO.
"""
        if cls._intents is None:
            current_dir = os.path.dirname(__file__)
            path = os.path.join(current_dir, 'intent_examples.json')
            with open(path, 'r', encoding='utf-8') as f:
                cls._intents = json.load(f)
        return cls._intents
