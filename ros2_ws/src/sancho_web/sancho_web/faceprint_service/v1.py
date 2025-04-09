import json

from typing import Optional, List
from fastapi import APIRouter, HTTPException, Query, Request, Path
from fastapi.responses import JSONResponse

from .faceprint_model import Faceprint, FaceprintDeleteResponse, FaceprintUpdate
from .api_utils import APIUtils

from dotenv import load_dotenv
from std_msgs.msg import String

load_dotenv()

router = APIRouter()

endpoint_name = "faceprints"
version = "v1"

api_node = None
def set_api_node(node):
    global api_node
    api_node = node

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
        #token = request.headers.get("Authorization")
        #if not token:
        #    raise HTTPException(status_code=400, detail="Inicia sesión para realizar esta acción")
        
        #query = build_query(email=email)
        #projection = APIUtils.build_projection(fields)
        #sort_criteria = APIUtils.build_sort_criteria(sort)

        faceprints = api_node.get_faceprint_request()
        
        total_count = len(faceprints)

        if hateoas:
            for faceprint in faceprints:
                faceprint["href"] = f"/api/{version}/{endpoint_name}/{faceprint['name']}"

        return JSONResponse(
            status_code=200,
            content=faceprints,
            headers={"Accept-Encoding": "gzip", "X-Total-Count": str(total_count)}
        )
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
        #token = request.headers.get("Authorization")
        #if not token:
        #    raise HTTPException(status_code=400, detail="Inicia sesión para realizar esta acción")

        #projection = APIUtils.build_projection(fields)

        #marker = DatabaseConnection.read_document("marker", id, projection)
        faceprint = api_node.get_faceprint_request(json.dumps({ "name": name }))
        if faceprint is None:
            return JSONResponse(status_code=404, content={"detail": f"Faceprint con nombre {name} no encontrado"})
        
        return JSONResponse(
            status_code=200,
            content=faceprint,
            headers={"Content-Type": "application/json", "X-Total-Count": "1"}
        )
    except HTTPException as e:
        raise e 
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al obtener el faceprint: {str(e)}")

@router.put("/{name}", tags=["Faceprints CRUD endpoints"], response_model=Faceprint)
async def update_item(
    name: str,
    faceprint_update: FaceprintUpdate,
    request: Request
):
    APIUtils.check_accept_json(request)
    APIUtils.check_content_type_json(request)

    try:
        #token = request.headers.get("Authorization")
        #if not token:
        #    raise HTTPException(status_code=400, detail="Inicia sesión para realizar esta acción")

        #decoded_token = auth.verify_id_token(token.split(" ")[1], clock_skew_seconds=10)
        #email = decoded_token.get("email")

        #existing_item = DatabaseConnection.read_document(endpoint_name, id)

        #if existing_item.get("email") != email:
        #    raise HTTPException(status_code=403, detail="No puedes editar este item, no eres el propietario")

        update_data = faceprint_update.model_dump(exclude_defaults=True)
        new_name = update_data["name"]
        if new_name is None:
            raise HTTPException(status_code=400, detail=f"No has incluido un campo name.")

        result, message = api_node.training_request(String(data="rename_class"), String(data=json.dumps({
            "class_name": name,
            "new_name": new_name,
        })))
        if result <= 0:
            raise HTTPException(status_code=400, detail=message)
        
        updated_item = api_node.get_faceprint_request(json.dumps({ "name": new_name }))
        return JSONResponse(
            status_code=200,
            content=updated_item,
            headers={"Content-Type": "application/json"}
        )
    #except auth.ExpiredIdTokenError:
    #    raise HTTPException(status_code=401, detail="Su sesión ha caducado. Por favor, inicia sesión de nuevo")
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al actualizar el faceprint: {str(e)}")


@router.delete("/{name}", tags=["Faceprints CRUD endpoints"], response_model=FaceprintDeleteResponse)
async def delete_item(
    name: str,
    request: Request
):
    APIUtils.check_accept_json(request)

    try:
        #token = request.headers.get("Authorization")
        #if not token:
        #    raise HTTPException(status_code=400, detail="Inicia sesión para realizar esta acción")

        #decoded_token = auth.verify_id_token(token.split(" ")[1], clock_skew_seconds=10)
        #email = decoded_token.get("email")

        result, message = api_node.training_request(String(data="delete_class"), String(data=json.dumps({
            "class_name": name
        })))
        if result <= 0:
            raise HTTPException(status_code=400, detail=message)

        return JSONResponse(
            status_code=200,
            content=f"Faceprint con nombre {name} elliminado correctamente."
        )
    #except auth.ExpiredIdTokenError:
    #    raise HTTPException(status_code=401, detail="Su sesión ha caducado. Por favor, inicia sesión de nuevo")
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al eliminar el item: {str(e)}")

