from enum import Enum

class PROVIDER(str, Enum):
    OPENAI = "openai"
    LLAMA = "llama"

# OpenAI
class OPENAI_LLM_MODEL(str, Enum):
    GPT_3_5_TURBO = "gpt-3.5-turbo"
    GPT_4 = "gpt-4"
    GPT_4O = "gpt-4o"

class OPENAI_EMBEDDING_MODEL(str, Enum):
    SMALL_3 = "text-embedding-3-small"
    ADA_002 = "text-embedding-ada-002" 
    BABBAGE_001 = "text-embedding-babbage-001"

# LLaMa
class LLAMA_LLM_MODEL(str, Enum):
    MISTRAL_7B = "mistralai/Mistral-7B-Instruct-v0.1"
    LLAMA3_8B = "meta-llama/Meta-Llama-3-8B-Instruct"

class LLAMA_EMBEDDING_MODEL(str, Enum):
    MINI_LM_L6_V2 = "sentence-transformers/all-MiniLM-L6-v2"
