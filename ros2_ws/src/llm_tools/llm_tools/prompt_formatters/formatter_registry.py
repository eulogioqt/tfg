from ..constants import PROVIDER
from . import OpenAIFormatter, HFFormatter, GeminiFormatter, PromptFormatter

FORMATTER_REGISTRY: dict[PROVIDER, PromptFormatter] = {
    PROVIDER.OPENAI: OpenAIFormatter(),
    PROVIDER.LLAMA: HFFormatter(),
    PROVIDER.MISTRAL: HFFormatter(),
    PROVIDER.QWEN: HFFormatter(),
    PROVIDER.DEEPSEEK: HFFormatter(),
    PROVIDER.PHI: HFFormatter(),
    PROVIDER.GEMINI: GeminiFormatter(),
}
