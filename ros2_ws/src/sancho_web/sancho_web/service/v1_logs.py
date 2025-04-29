from typing import Optional, List
from fastapi import APIRouter, HTTPException, Query, Request, Path

from .log_model import Log
from .api_utils import APIUtils
from ..interfaces import LogAPIInterface

from dotenv import load_dotenv

load_dotenv()

router = APIRouter()

endpoint_name = "logs"
version = "v1"

log_interface: LogAPIInterface = None
def set_log_api(interface):
    global log_interface
    log_interface = interface

@router.get("", tags=["Logs CRUD endpoints"], response_model=List[Log])
async def get_logs(
    request: Request,
    faceprint_id: Optional[str] = Query(None, description="ID de un faceprint"),
    fields: Optional[str] = Query(None, description="Campos específicos a devolver"),
    sort: Optional[str] = Query(None, description="Campos por los que ordenar, separados por comas"),
    offset: int = Query(default=0, description="Índice de inicio para los resultados de la paginación"),
    limit: int = Query(default=10, description="Cantidad de marcadores a devolver, por defecto 10"),
    hateoas: Optional[bool] = Query(default=False, description="Incluir enlace HATEOAS")
):
    APIUtils.check_accept_json(request)

    try:
        response = log_interface.get_all_logs(**({ "faceprint_id": faceprint_id} if faceprint_id else {}))
        
        return response.to_fastapi()
    
    except HTTPException as e:
        raise e 
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al buscar los logs: {str(e)}")

@router.get("/{id}", tags=["Logs CRUD endpoints"], response_model=Log)
async def get_logs_by_id(
    request: Request,
    id: str = Path(description="Id del log"),
    fields: Optional[str] = Query(None, description="Campos específicos a devolver"),
    sort: Optional[str] = Query(None, description="Campos por los que ordenar, separados por comas"),
    offset: int = Query(default=0, description="Índice de inicio para los resultados de la paginación"),
    limit: int = Query(default=10, description="Cantidad de marcadores a devolver, por defecto 10"),
    hateoas: Optional[bool] = Query(default=False, description="Incluir enlace HATEOAS")
):
    APIUtils.check_accept_json(request)
    
    try:
        response = log_interface.get_log_by_id(id)

        return response.to_fastapi()
    
    except HTTPException as e:
        raise e 
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al obtener el log: {str(e)}")
