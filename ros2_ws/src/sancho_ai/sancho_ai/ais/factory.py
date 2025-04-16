from enum import Enum
from typing import Type

from .interfaces.ai import AI
from .simple_ai import SimpleAI
from .classification_templates_ai import ClassificationTemplatesAI


class AIType(str, Enum):
    SIMPLE = "SIMPLE"
    CLASSIFICATION_TEMPLATES = "CLASSIFICATION_TEMPLATES"


AI_CLASSES: dict[AIType, Type[AI]] = {
    AIType.SIMPLE: SimpleAI,
    AIType.CLASSIFICATION_TEMPLATES: ClassificationTemplatesAI
}

def create_sancho_ai(ai_type: AIType) -> AI:
    AI_CLASS = AI_CLASSES.get(ai_type)
    if not AI_CLASS:
        raise ValueError(f"AI type '{ai_type}' is not registered.")
    
    return AI_CLASS()
