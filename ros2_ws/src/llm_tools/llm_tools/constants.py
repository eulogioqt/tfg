from enum import Enum

class PROVIDER(str, Enum):
    OPENAI = "openai"
    LLAMA = "llama"
    MISTRAL = "mistral"
    PHI = "phi"
    QWEN = "qwen"
    DEEPSEEK = "deepseek"
    GEMINI = "gemini"
    SBERT = "sbert"
    E5 = "e5"
    BAAI = "baai"

# OpenAI
class OPENAI_LLM_MODEL(str, Enum):
    GPT_3_5_TURBO = "gpt-3.5-turbo"
    GPT_4 = "gpt-4"
    GPT_4O = "gpt-4o"

class OPENAI_EMBEDDING_MODEL(str, Enum):
    SMALL_3 = "text-embedding-3-small"
    ADA_002 = "text-embedding-ada-002" 
    BABBAGE_001 = "text-embedding-babbage-001"
    LARGE_3 = "text-embedding-3-large"

# LLaMa (Meta)
class LLAMA_LLM_MODEL(str, Enum):
    LLAMA3_8B = "meta-llama/Meta-Llama-3-8B-Instruct"

# Mistral
class MISTRAL_LLM_MODEL(str, Enum):
    MISTRAL_7B = "mistralai/Mistral-7B-Instruct-v0.1"
    MIXTRAL_8X7B = "mistralai/Mixtral-8x7B-Instruct-v0.1"

# Phi
class PHI_LLM_MODEL(str, Enum):
    PHI_2 = "microsoft/phi-2"

# Qwen
class QWEN_LLM_MODEL(str, Enum):
    QWEN_7B = "Qwen/Qwen1.5-7B-Chat"

class QWEN_EMBEDDING_MODEL(str, Enum):
    QWEN_EMBED = "Qwen/Qwen1.5-7B-Chat-Embedding"

# DeepSeek
class DEEPSEEK_LLM_MODEL(str, Enum):
    DEEPSEEK_CHAT = "deepseek-ai/deepseek-llm-7b-chat"

class DEEPSEEK_EMBEDDING_MODEL(str, Enum):
    DEEPSEEK_EMBED = "deepseek-ai/deepseek-embedding"

# Gemini (Google)
class GEMINI_LLM_MODEL(str, Enum):
    GEMINI_FLASH = "gemini-1.5-flash-latest"

# SBERT
class SBERT_EMBEDDING_MODEL(str, Enum):
    MINI_LM_L6_V2 = "sentence-transformers/all-MiniLM-L6-v2"

# E5
class E5_EMBEDDING_MODEL(str, Enum):
    E5_LARGE_V2 = "intfloat/e5-large-v2"

# BAAI (BGE)
class BAAI_EMBEDDING_MODEL(str, Enum):
    BGE_M3 = "BAAI/bge-m3"
    BGE_BASE_EN = "BAAI/bge-base-en-v1.5"
