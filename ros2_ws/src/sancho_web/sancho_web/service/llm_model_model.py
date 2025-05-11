from pydantic import BaseModel, Field
from typing import Optional


class LLMModel(BaseModel):
    provider: str = Field(example="openai")
    model: str = Field(example="gpt-3.5-turbo")
    needs_api_key: bool = Field(example=True)
    loaded: bool = Field(example=True)
    active: bool = Field(example=False)

class LLMLoadModel(BaseModel):
    provider: str = Field(example="openai")
    model: str = Field(example="gpt-3.5-turbo")
    api_key: Optional[str] = Field(default=None, example="sk-...")

class LLMUnloadModel(BaseModel):
    provider: str = Field(example="openai")
    model: str = Field(example="gpt-3.5-turbo")

class LLMActiveModel(BaseModel):
    provider: str = Field(example="openai")
    model: str = Field(example="gpt-3.5-turbo")

class LLMResult(BaseModel):
    success: bool = Field(example=True)
    message: str = Field(example="Modelo activado correctamente")
