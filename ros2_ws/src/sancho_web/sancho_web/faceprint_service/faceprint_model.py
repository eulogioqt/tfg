from pydantic import BaseModel, Field
from datetime import datetime

class Faceprint(BaseModel):
    name: str = Field(example="Juan")
    features: str = Field(example='[0.1, 0.2, 0.3, ...]')
    size: str = Field(example='{"width": 128, "height": 128}')
    learning_date: str = Field(default=str(datetime.now().timestamp()), example="1712345678")
    face: str = Field(example="data:image/jpeg;base64,...")

class FaceprintCreate(BaseModel):
    name: str = Field(example="Juan")
    face: str = Field(example="data:image/jpeg;base64,...")

class FaceprintUpdate(BaseModel):
    name: str = Field(example="Juan")

class FaceprintDeleteResponse(BaseModel):
    details: str = "El registro facial se ha borrado correctamente."
