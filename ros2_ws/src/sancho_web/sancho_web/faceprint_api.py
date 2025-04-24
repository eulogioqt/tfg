import json

from std_msgs.msg import String

from .client_node import ClientNode
from .interfaces import FaceprintAPIInterface, HTTPException, JSONResponse


class FaceprintAPI(FaceprintAPIInterface):

    def __init__(self):
        self.node = ClientNode()

    def get_all_faceprints(self):
        faceprints_json = self.node.get_faceprint_request()
        faceprints = json.loads(faceprints_json)

        return JSONResponse(content=faceprints)

    def get_faceprint(self, name):
        faceprint_json = self.node.get_faceprint_request(json.dumps({ "name": name }))

        faceprint = json.loads(faceprint_json)
        if faceprint is None:
            return HTTPException(status_code=404, detail=f"Faceprint con nombre {name} no encontrado")
        
        return JSONResponse(content=faceprint)

    def create_faceprint(self, name, image_base64):
        image_cv2 = self.node.br.base64_to_cv2(image_base64)
        image_msg = self.node.br.cv2_to_imgmsg(image_cv2, encoding="bgr8")
        
        positions_msg, scores_msg = self.node.detection_request(image_msg)
        positions, scores = self.node.br.msg_to_detector(positions_msg, scores_msg)

        if len(positions) == 0:
            return HTTPException(detail=f"No se ha detectado ningún rostro en la imagen.")
        elif len(positions) > 1:
            return HTTPException(detail=f"Se ha detectado más de un rostro en la imagen. Solo debe haber uno.")
        else:
            position = positions_msg[0]
            score = scores[0]

            if score < 1:
                return HTTPException(detail=f"La puntuación de la detección ha sido demasiado baja ({score:.2f} < 1). Por favor, envía una imagen mejor")
            else:
                face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg = \
                    self.node.recognition_request(image_msg, position, score)
                face_aligned, features, classified, distance, pos = \
                    self.node.br.msg_to_recognizer(face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg)
                
                LOWER_BOUND = 0.75
                MIDDLE_BOUND = 0.80
                if distance < LOWER_BOUND:
                    face_aligned_base64 = self.node.br.cv2_to_base64(face_aligned)
                    already_known, message = self.node.training_request(String(data="add_class"), String(data=json.dumps({
                        "class_name": name,
                        "features": features,
                        "face": face_aligned_base64,
                        "score": score
                    }))) # Añadimos clase (en teoria es alguien nuevo)
                    if already_known < 0:
                        return HTTPException(detail=message)
                    else: # Añade vector de característica
                        updated_item_json = self.node.get_faceprint_request(json.dumps({ "name": name }))
                        updated_item = json.loads(updated_item_json)

                        return JSONResponse((208 if already_known == 1 else 200), updated_item)
                    
                elif distance < MIDDLE_BOUND:
                    return HTTPException(detail=f"Nunca pensé que pasase esto MIDDLE BOUND.")
                else:
                    return HTTPException(detail=f"Ya te conozco {classified}, esta función es para personas nuevas.")

    def update_faceprint(self, name, faceprint):
        new_name = faceprint["name"]
        if new_name is None:
            return HTTPException(detail="No has incluido el campo name.")

        result, message = self.node.training_request(String(data="rename_class"), String(data=json.dumps({
            "class_name": name,
            "new_name": new_name,
        })))
        if result <= 0:
            return HTTPException(detail=message)
        
        updated_item_json = self.node.get_faceprint_request(json.dumps({ "name": new_name }))
        updated_item = json.loads(updated_item_json)

        return JSONResponse(content=updated_item)

    def delete_faceprint(self, name):
        result, message = self.node.training_request(String(data="delete_class"), String(data=json.dumps({
            "class_name": name
        })))
        if result <= 0:
            return HTTPException(detail=message)

        return JSONResponse(content=f"Faceprint con nombre {name} elliminado correctamente.")