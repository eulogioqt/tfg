"""TODO: Add module documentation."""
from pydantic import BaseModel, Field
from datetime import datetime

class Faceprint(BaseModel):
"""TODO: Describe class."""
    id: str = Field(example="0")
    name: str = Field(example="Juan")
    features: str = Field(example='[0.1, 0.2, 0.3, ...]')
    size: str = Field(example='[13]')
    learning_date: str = Field(default=str(datetime.now().timestamp()), example="1712345678")
    face: str = Field(example="...")

class FaceprintCreate(BaseModel):
"""TODO: Describe class."""
    name: str = Field(example="Juan")
    image: str = Field(example="...")

class FaceprintUpdate(BaseModel):
"""TODO: Describe class."""
    name: str = Field(example="Juan")

class FaceprintDeleteResponse(BaseModel):
"""TODO: Describe class."""
    details: str = "El registro facial se ha borrado correctamente."
