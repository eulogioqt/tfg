from enum import Enum

from .providers.openai_provider import OpenAIProvider
from .providers.mistral_provider import MistralProvider
from .providers.phi_provider import PhiProvider
from .providers.qwen_provider import QwenProvider
from .providers.deepseek_provider import DeepSeekProvider
from .providers.gemini_provider import GeminiProvider
from .providers.sbert_provider import SBERTProvider
from .providers.e5_provider import E5Provider
from .providers.baai_provider import BAAIProvider

class SmartStrEnum(str, Enum):
    def __str__(self):
        return self.value

    def __repr__(self):
        return self.value

class PROVIDER(SmartStrEnum):
    OPENAI = "openai"
    MISTRAL = "mistral"
    PHI = "phi"
    QWEN = "qwen"
    DEEPSEEK = "deepseek"
    GEMINI = "gemini"
    SBERT = "sbert"
    E5 = "e5"
    BAAI = "baai"

PROVIDER_CLASS_MAP = {
    PROVIDER.OPENAI: OpenAIProvider,
    PROVIDER.MISTRAL: MistralProvider,
    PROVIDER.PHI: PhiProvider,
    PROVIDER.QWEN: QwenProvider,
    PROVIDER.DEEPSEEK: DeepSeekProvider,
    PROVIDER.GEMINI: GeminiProvider,
    PROVIDER.SBERT: SBERTProvider,
    PROVIDER.E5: E5Provider,
    PROVIDER.BAAI: BAAIProvider,
}

class MODELS: # hacer el research para tener lo ultimo de lo ultmo, por ejemplo qwen tiene uno mas moderno
    class LLM:
        class OPENAI(SmartStrEnum):
            GPT_3_5_TURBO = "gpt-3.5-turbo"
            GPT_4 = "gpt-4"
            GPT_4O = "gpt-4o"

        class MISTRAL(SmartStrEnum):
            MISTRAL_7B = "mistralai/Mistral-7B-Instruct-v0.1"
            MIXTRAL_8X7B = "mistralai/Mixtral-8x7B-Instruct-v0.1"

        class PHI(SmartStrEnum):
            PHI_2 = "microsoft/phi-2"

        class QWEN(SmartStrEnum): # Qwen/Qwen2.5-Omni-7B
            QWEN_7B = "Qwen/Qwen1.5-7B-Chat"

        class DEEPSEEK(SmartStrEnum):
            DEEPSEEK_CHAT = "deepseek-ai/deepseek-llm-7b-chat"

        class GEMINI(SmartStrEnum):
            GEMINI_FLASH = "gemini-1.5-flash-latest"

    class EMBEDDING:
        class OPENAI(SmartStrEnum):
            SMALL_3 = "text-embedding-3-small"
            ADA_002 = "text-embedding-ada-002"
            BABBAGE_001 = "text-embedding-babbage-001"
            LARGE_3 = "text-embedding-3-large"

        class QWEN(SmartStrEnum):
            QWEN_EMBED = "Alibaba-NLP/gte-Qwen2-7B-instruct"

        class DEEPSEEK(SmartStrEnum):
            DEEP_EMBED = "deepseek-ai/deepseek-coder-6.7b-instruct"

        class SBERT(SmartStrEnum):
            MINI_LM_L6_V2 = "sentence-transformers/all-MiniLM-L6-v2"

        class E5(SmartStrEnum):
            E5_LARGE_V2 = "intfloat/e5-large-v2"

        class BAAI(SmartStrEnum):
            BGE_M3 = "BAAI/bge-m3"
            BGE_BASE_EN = "BAAI/bge-base-en-v1.5"