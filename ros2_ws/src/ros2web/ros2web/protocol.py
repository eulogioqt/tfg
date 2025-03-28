import cv2
import json
import base64

from enum import Enum

##### CLIENT MESSAGES #####
class PromptMessage():
    def __init__(self, msg):
        self.id = msg["id"]
        self.value = msg["value"]

##### SERVER MESSAGES #####
class DisplayDataMessage():
    def __init__(self, data, type):
        self.data = DISPLAY_DATA_OBJECT[type](data)

    def to_json(self):       
        message = {
            "type": MessageType.DISPLAY_DATA,
            "data": self.data.to_dict()
        }
    
        return json.dumps(message)

class ResponseMessage():
    def __init__(self, id, response):
        self.id = id
        self.response = response
    
    def to_json(self):
        message = {
            "type": MessageType.RESPONSE,
            "data": {
                "id": self.id,
                "value": self.response
            }
        }

        return json.dumps(message)

##### SERVER DISPLAY DATA MESSAGES #####
class ImageMessage():
    def __init__(self, image):
        ret, jpeg = cv2.imencode('.jpg', image)

        if ret:
            self.frame_base64 = base64.b64encode(jpeg.tobytes()).decode('utf-8')
        else:
            raise Exception("Error on protocol.image_message (not ret)")

    def to_dict(self):
        message = {
            "type": DisplayDataType.IMAGE,
            "value": self.frame_base64
        }

        return message

##### TYPES #####
class MessageType(str, Enum):
    RESPONSE = "RESPONSE"
    PROMPT = "PROMPT"
    DISPLAY_DATA = "DISPLAY_DATA"

class DisplayDataType(str, Enum):
    IMAGE = "IMAGE"
    AUDIO = "AUDIO" # Para mostrar en la interfaz web lo que tu hablas
    OTHER = "OTHER"
    
MESSAGE_OBJECT = {
    MessageType.RESPONSE: ResponseMessage,
    MessageType.PROMPT: PromptMessage,
    MessageType.DISPLAY_DATA: DisplayDataMessage
}

DISPLAY_DATA_OBJECT = {
    DisplayDataType.IMAGE: ImageMessage
}

##### PARSER #####
def parse_message(msg):
    try:
        msg = json.loads(msg)
        return msg.get("type"), msg.get("data")
    except json.JSONDecodeError:
        print("Error: mensaje JSON malformado")
        return None, None
