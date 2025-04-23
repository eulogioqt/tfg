import json

from typing import Optional, List
from fastapi import APIRouter, HTTPException, Query, Request, Path
from fastapi.responses import JSONResponse

from .faceprint_model import Faceprint, FaceprintCreate, FaceprintUpdate, FaceprintDeleteResponse
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

        image_cv2 = api_node.br.base64_to_cv2(image_base64)
        image_msg = api_node.br.cv2_to_imgmsg(image_cv2, encoding="bgr8")
        
        positions_msg, scores_msg = api_node.detection_request(image_msg)
        positions, scores = api_node.br.msg_to_detector(positions_msg, scores_msg)

        if len(positions) == 0:
            raise HTTPException(status_code=400, detail=f"No se ha detectado ningún rostro en la imagen.")
        elif len(positions) > 1:
            raise HTTPException(status_code=400, detail=f"Se ha detectado más de un rostro en la imagen. Solo debe haber uno.")
        else:
            position = positions_msg[0]
            score = scores_msg[0]

            if score < 1:
                raise HTTPException(status_code=400, detail=f"La puntuación de la detección ha sido demasiado baja ({score:.2f} < 1). Por favor, envía una imagen mejor")
            else:
                face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg = \
                    api_node.recognition_request(image_msg, position, score)
                face_aligned, features, classified, distance, pos = \
                    api_node.br.msg_to_recognizer(face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg)
                
                LOWER_BOUND = 0.75
                MIDDLE_BOUND = 0.80
                if distance < LOWER_BOUND:
                    face_aligned_base64 = api_node.br.cv2_to_base64(face_aligned)
                    already_known, message = api_node.training_request(String(data="add_class"), String(data=json.dumps({
                        "class_name": name,
                        "features": features,
                        "face": face_aligned_base64,
                        "score": score
                    }))) # Añadimos clase (en teoria es alguien nuevo)
                    if already_known < 0:
                        raise HTTPException(status_code=400, detail=message)
                    elif already_known == 1:
                        raise HTTPException(status_code=208, detail=f"Ya te conocía {classified}, pero he reforzado mi aprendizaje.")
                    elif already_known == 0:
                        updated_item = api_node.get_faceprint_request(json.dumps({ "name": name }))
                        
                        return JSONResponse(
                            status_code=200,
                            content=updated_item,
                            headers={"Content-Type": "application/json"}
                        )
                    
                elif distance < MIDDLE_BOUND:
                    raise HTTPException(status_code=400, detail=f"Nunca pensé que pasase esto MIDDLE BOUND.")
                else:
                    raise HTTPException(status_code=400, detail=f"Ya te conozco {classified}, esta función es para personas nuevas.")
        
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
async def delete_faceprint(
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

