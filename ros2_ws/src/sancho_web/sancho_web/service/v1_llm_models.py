from typing import Optional, List
from fastapi import APIRouter, HTTPException, Query, Request, Path

from .llm_model_model import LLMModel, LLMLoadModel, LLMUnloadModel, LLMActiveModel, LLMResult
from .api_utils import APIUtils
from ..apis import LLMModelAPI

from dotenv import load_dotenv

load_dotenv()

router = APIRouter()

endpoint_name = "llm_models"
version = "v1"

llm_model_api: LLMModelAPI = None
def set_llm_model_api(api):
    global llm_model_api
    llm_model_api = api


@router.get("", tags=["LLM Models CRUD endpoints"], response_model=List[LLMModel])
async def get_llm_models(
    request: Request,
    providers: Optional[List[str]] = Query(None, description="Lista de proveedores a buscar"),
    fields: Optional[str] = Query(None, description="Campos específicos a devolver"),
    sort: Optional[str] = Query(None, description="Campos por los que ordenar, separados por comas"),
    offset: int = Query(default=0, description="Índice de inicio para los resultados de la paginación"),
    limit: int = Query(default=10, description="Cantidad de modelos a devolver, por defecto 10"),
    hateoas: Optional[bool] = Query(default=False, description="Incluir enlace HATEOAS")
):
    APIUtils.check_accept_json(request)

    try:
        response = llm_model_api.get_all_llm_models(**({"providers": providers} if providers else {}))
        return response.to_fastapi()

    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(status_code=500, detail={str(e)})


@router.get("/{provider}/{model}", tags=["LLM Models CRUD endpoints"], response_model=LLMModel)
async def get_llm_model_by_name(
    request: Request,
    provider: str = Path(description="Nombre del proveedor"),
    model: str = Path(description="Nombre del modelo"),
    fields: Optional[str] = Query(None, description="Campos específicos a devolver"),
    sort: Optional[str] = Query(None, description="Campos por los que ordenar, separados por comas"),
    offset: int = Query(default=0, description="Índice de inicio para los resultados de la paginación"),
    limit: int = Query(default=10, description="Cantidad de modelos a devolver, por defecto 10"),
    hateoas: Optional[bool] = Query(default=False, description="Incluir enlace HATEOAS")
):
    APIUtils.check_accept_json(request)

    try:
        response = llm_model_api.get_llm_model(provider, model)
        
        return response.to_fastapi()

    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(status_code=500, detail={str(e)})


@router.post("/load", tags=["LLM Models CRUD endpoints"], response_model=LLMResult)
async def load_llm_model(
    llm_load_model: LLMLoadModel,
    request: Request
):
    APIUtils.check_accept_json(request)
    APIUtils.check_content_type_json(request)

    try:
        data = llm_load_model.model_dump(exclude_defaults=True)
        provider = data.get("provider")
        model = data.get("model")
        api_key = data.get("api_key", "")

        response = llm_model_api.load_llm_model(provider, model, api_key)

        return response.to_fastapi()

    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(status_code=500, detail={str(e)})


@router.post("/unload", tags=["LLM Models CRUD endpoints"], response_model=LLMResult)
async def unload_llm_model(
    llm_unload_model: LLMUnloadModel,
    request: Request
):
    APIUtils.check_accept_json(request)
    APIUtils.check_content_type_json(request)

    try:
        data = llm_unload_model.model_dump(exclude_defaults=True)
        provider = data.get("provider")
        model = data.get("model")

        response = llm_model_api.unload_llm_model(provider, model)

        return response.to_fastapi()

    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(status_code=500, detail={str(e)})


@router.post("/activate", tags=["LLM Models CRUD endpoints"], response_model=LLMResult)
async def activate_llm_model(
    llm_active_model: LLMActiveModel,
    request: Request
):
    APIUtils.check_accept_json(request)
    APIUtils.check_content_type_json(request)

    try:
        data = llm_active_model.model_dump(exclude_defaults=True)
        provider = data.get("provider")
        model = data.get("model")

        response = llm_model_api.set_active_llm_model(provider, model)

        return response.to_fastapi()

    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(status_code=500, detail={str(e)})
