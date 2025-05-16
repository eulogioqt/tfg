from enum import Enum
from typing import Type

from .ai import AI

from .dummy_ai import DummyAI
from .simple_ai import SimpleAI
from .classification_templates_ai import ClassificationTemplatesAI
from .classification_generation_ai import ClassificationGenerationAI

class SmartStrEnum(str, Enum):
    def __str__(self):
        return self.value

    def __repr__(self):
        return self.value
    
class AIType(SmartStrEnum):
    DUMMY = "DUMMY"
    SIMPLE = "SIMPLE"
    CLASSIFICATION_TEMPLATES = "CLASSIFICATION_TEMPLATES"
    CLASSIFICATION_GENERATION = "CLASSIFICATION_GENERATION"


AI_CLASSES: dict[AIType, Type[AI]] = {
    AIType.DUMMY: DummyAI,
    AIType.SIMPLE: SimpleAI,
    AIType.CLASSIFICATION_TEMPLATES: ClassificationTemplatesAI,
    AIType.CLASSIFICATION_GENERATION: ClassificationGenerationAI
}

def create_sancho_ai(ai_type: AIType) -> AI:
    AI_CLASS = AI_CLASSES.get(ai_type)
    if not AI_CLASS:
        raise ValueError(f"AI type '{ai_type}' is not registered.")
    
    return AI_CLASS()
