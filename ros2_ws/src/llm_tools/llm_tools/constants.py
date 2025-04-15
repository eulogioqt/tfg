from enum import Enum

class PROVIDER(str, Enum):
    OPENAI = "openai"

class OPENAI_MODEL(str, Enum):
    GPT_35_TURBO = "gpt-3.5-turbo"
    GPT_4 = "gpt-4"
    GPT_4O = "gpt-4o"

