from unittest.mock import Mock

from sancho_ai.ais.base.modular_ai import ModularAI
from sancho_ai.prompts.commands.commands import COMMANDS


class DummyModularAI(ModularAI):
    pass


# Hybrid: pipeline when intent recognized

def test_modular_ai_pipeline_known_intent():
    classifier = Mock()
    executor = Mock()
    generator = Mock()

    classifier.classify.return_value = (COMMANDS.TAKE_PICTURE, {"foo": 1}, "prov", "model")
    executor.execute.return_value = ("details", "ok", {"data": 42})
    generator.generate_response.return_value = ("done", "happy", "gprov", "gmodel")

    ai = DummyModularAI(classifier, executor, generator)
    value, intent, args, prov, model = ai.on_message("hi", ["h"])

    classifier.classify.assert_called_once_with("hi", ["h"])
    executor.execute.assert_called_once_with(COMMANDS.TAKE_PICTURE, {"foo": 1})
    generator.generate_response.assert_called_once()
    assert value == {"text": "done", "emotion": "happy", "data": {"data": 42}}
    assert prov == "gprov" and model == "gmodel"
    assert intent == COMMANDS.TAKE_PICTURE and args == {"foo": 1}


# Hybrid: pipeline when intent is unknown

def test_modular_ai_pipeline_unknown_intent():
    classifier = Mock()
    executor = Mock()
    generator = Mock()

    classifier.classify.return_value = (COMMANDS.UNKNOWN, {}, "prov", "model")
    generator.continue_conversation.return_value = ("ok", "neutral", "gprov", "gmodel")

    ai = DummyModularAI(classifier, executor, generator)
    value, intent, args, prov, model = ai.on_message("hello")

    executor.execute.assert_not_called()
    generator.continue_conversation.assert_called_once()
    assert intent == COMMANDS.UNKNOWN
    assert value["text"] == "ok"
