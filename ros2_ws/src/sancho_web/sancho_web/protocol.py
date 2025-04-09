import json

from enum import Enum


##### CLIENT MESSAGES #####
class PromptMessage():
    def __init__(self, msg):
        self.id = msg["id"]
        self.value = msg["value"]


##### SERVER MESSAGES #####
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

class FaceprintEventMessage():
    class Event(str, Enum):
        CREATE = "CREATE"
        UPDATE = "UPDATE"
        DELETE = "DELETE"

    def __init__(self, event, name):
        self.event = event
        self.name = name
    
    def to_json(self):
        message = {
            "type": MessageType.FACEPRINT_EVENT,
            "data": {
                "event": self.event,
                "name": self.name
            }
        }

        return json.dumps(message)


##### TYPES #####
class MessageType(str, Enum):
    RESPONSE = "RESPONSE"
    PROMPT = "PROMPT"
    FACEPRINT_EVENT = "FACEPRINT_EVENT"
    
MESSAGE_OBJECT = {
    MessageType.RESPONSE: ResponseMessage,
    MessageType.PROMPT: PromptMessage,
    MessageType.FACEPRINT_EVENT: FaceprintEventMessage
}


##### PARSER #####
def parse_message(msg):
    try:
        msg = json.loads(msg)
        return msg.get("type"), msg.get("data")
    except json.JSONDecodeError:
        print("Error: mensaje JSON malformado")
        return None, None
