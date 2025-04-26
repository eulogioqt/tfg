import json

from typing import Optional, List
from fastapi import APIRouter, HTTPException, Query, Request, Path
from fastapi.responses import JSONResponse, Response

from .faceprint_model import Faceprint, FaceprintCreate, FaceprintUpdate, FaceprintDeleteResponse
from .api_utils import APIUtils
from ..interfaces import FaceprintAPIInterface

from dotenv import load_dotenv
from std_msgs.msg import String

load_dotenv()

router = APIRouter()

endpoint_name = "faceprints"
version = "v1"

faceprint_interface: FaceprintAPIInterface = None
def set_faceprint_api(interface):
    global faceprint_interface
    faceprint_interface = interface

@router.get("", tags=["Faceprints CRUD endpoints"], response_model=List[Faceprint])
async def get_faceprints(
    request: Request,
    name: Optional[str] = Query(None, description="Nombre de la persona"),
    fields: Optional[str] = Query(None, description="Campos específicos a devolver"),
    sort: Optional[str] = Query(None, description="Campos por los que ordenar, separados por comas"),
    offset: int = Query(default=0, description="Índice de inicio para los resultados de la paginación"),
    limit: int = Query(default=10, description="Cantidad de marcadores a devolver, por defecto 10"),
    hateoas: Optional[bool] = Query(default=False, description="Incluir enlace HATEOAS")
):
    APIUtils.check_accept_json(request)

    try:
        response = faceprint_interface.get_all_faceprints()
        
        return response.to_fastapi()
    
    except HTTPException as e:
        raise e 
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al buscar los faceprints: {str(e)}")

@router.get("/{name}", tags=["Faceprints CRUD endpoints"], response_model=Faceprint)
async def get_faceprint_by_name(
    request: Request,
    name: str = Path(description="Nombre de la persona"),
    fields: Optional[str] = Query(None, description="Campos específicos a devolver")
):
    APIUtils.check_accept_json(request)
    
    try:
        response = faceprint_interface.get_faceprint(name)

        return response.to_fastapi()
    
    except HTTPException as e:
        raise e 
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al obtener el faceprint: {str(e)}")

@router.post("", tags=["Faceprints CRUD endpoints"], response_model=FaceprintCreate)
async def create_faceprint(
    faceprint_create: FaceprintCreate,
    request: Request
):
    APIUtils.check_accept_json(request)
    APIUtils.check_content_type_json(request)

    try:
        update_data = faceprint_create.model_dump(exclude_defaults=True)
        name = update_data["name"]
        image_base64 = update_data["image"]
        
        response = faceprint_interface.create_faceprint(name, image_base64)

        return response.to_fastapi()

    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al actualizar el faceprint: {str(e)}")

@router.put("/{name}", tags=["Faceprints CRUD endpoints"], response_model=Faceprint)
async def update_faceprint(
    name: str,
    faceprint_update: FaceprintUpdate,
    request: Request
):
    APIUtils.check_accept_json(request)
    APIUtils.check_content_type_json(request)

    try:
        update_data = faceprint_update.model_dump(exclude_defaults=True)
        response = faceprint_interface.update_faceprint(name, update_data)

        return response.to_fastapi()
    
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al actualizar el faceprint: {str(e)}")


@router.delete("/{name}", tags=["Faceprints CRUD endpoints"], response_model=FaceprintDeleteResponse)
async def delete_faceprint(
    name: str,
    request: Request
):
    APIUtils.check_accept_json(request)

    try:
        response = faceprint_interface.delete_faceprint(name)

        return response.to_fastapi()
    
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al eliminar el item: {str(e)}")

