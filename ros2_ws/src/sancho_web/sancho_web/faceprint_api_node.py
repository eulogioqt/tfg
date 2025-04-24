import json
import rclpy
import uvicorn

from rclpy.node import Node
from std_msgs.msg import String

from hri_msgs.srv import Detection, Recognition, Training, GetString
from hri_vision.hri_bridge import HRIBridge

from .faceprint_service.app import app
from .faceprint_service.v1 import set_api_node
from .faceprint_database_interface import FaceprintDatabaseInterface, APIResponse, HTTPException, JSONResponse

class APIClientNode(Node):
    def __init__(self):
        super().__init__('api_client_node')

        self.get_faceprint_client = self.create_client(GetString, 'recognition/get_faceprint')
        while not self.get_faceprint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get All service not available, waiting again...')

        self.detection_client = self.create_client(Detection, 'detection')
        while not self.detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Detection service not available, waiting again...')
        
        self.recognition_client = self.create_client(Recognition, 'recognition')
        while not self.recognition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Recognition service not available, waiting again...')

        self.training_client = self.create_client(Training, 'recognition/training')
        while not self.training_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Training service not available, waiting again...')

        self.br = HRIBridge()
        self.get_logger().info("ROS Client Node initializated succesfully")

    def spin(self):
        set_api_node(self)

        uvicorn.run(app, host="localhost", port=7654)

    def get_faceprint_request(self, args_msg=""):
        get_all_request = GetString.Request()
        get_all_request.args = args_msg

        future_get_all = self.get_faceprint_client.call_async(get_all_request)
        rclpy.spin_until_future_complete(self, future_get_all)
        result_get_all = future_get_all.result()

        return result_get_all.text

    # Demasiada replica de hri logic, no?
    # Ademas cuando lo haga con el llm lo mismo, mas replica aun porque por ahi va por websocket a sancho ai....
    def detection_request(self, frame_msg):
        detection_request = Detection.Request()
        detection_request.frame = frame_msg

        future_detection = self.detection_client.call_async(detection_request)
        rclpy.spin_until_future_complete(self, future_detection)
        result_detection = future_detection.result()

        return result_detection.positions, result_detection.scores

    def recognition_request(self, frame_msg, position_msg, score_msg):
        recognition_request = Recognition.Request()
        recognition_request.frame = frame_msg
        recognition_request.position = position_msg
        recognition_request.score = score_msg

        future_recognition = self.recognition_client.call_async(recognition_request)
        rclpy.spin_until_future_complete(self, future_recognition)
        result_recognition = future_recognition.result()

        return (result_recognition.face_aligned, result_recognition.features,
                result_recognition.classified, result_recognition.distance, result_recognition.pos)    

    def training_request(self, cmd_type_msg, args_msg):
        training_request = Training.Request()
        training_request.cmd_type = cmd_type_msg
        training_request.args = args_msg
        training_request.origin = Training.Request.ORIGIN_WEB

        future_training = self.training_client.call_async(training_request)
        rclpy.spin_until_future_complete(self, future_training)
        result_training = future_training.result()

        return result_training.result, result_training.message.data

class APIClient(FaceprintDatabaseInterface):

    def __init__(self):
        self.node = APIClientNode()
    
    def spin(self):
        self.node.spin()

    def get_all_faces(self):
        faceprints_json = self.node.get_faceprint_request()
        faceprints = json.loads(faceprints_json)

        return JSONResponse(content=faceprints)

    def get_face(self, name):
        faceprint_json = self.node.get_faceprint_request(json.dumps({ "name": name }))

        faceprint = json.loads(faceprint_json)
        if faceprint is None:
            return HTTPException(status_code=404, detail=f"Faceprint con nombre {name} no encontrado")
        
        return JSONResponse(content=faceprint)

    def create_face(self, name, image_base64):
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
                        updated_item = self.node.get_faceprint_request(json.dumps({ "name": name }))
                        
                        return JSONResponse((208 if already_known == 1 else 200), updated_item)
                    
                elif distance < MIDDLE_BOUND:
                    return HTTPException(detail=f"Nunca pensé que pasase esto MIDDLE BOUND.")
                else:
                    return HTTPException(detail=f"Ya te conozco {classified}, esta función es para personas nuevas.")

    def update_face(self, name, faceprint):
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

def main(args=None):
    rclpy.init(args=args)

    api_node = APIClient()

    api_node.spin()
    rclpy.shutdown()

