from typing import Optional, List
from fastapi import APIRouter, HTTPException, Query, Request, Path

#from .tts_model_model import TTSModel
from .api_utils import APIUtils
from ..apis import TTSModelAPI

from dotenv import load_dotenv

load_dotenv()

router = APIRouter()

endpoint_name = "tts_models"
version = "v1"

tts_model_api: TTSModelAPI = None
def set_tts_model_api(api):
    global tts_model_api
    tts_model_api = api

@router.get("", tags=["Sessions CRUD endpoints"])#, response_model=List[Session])
async def get_sessions(
    request: Request,
):
    return { "data": "hola q tal chavales" }
