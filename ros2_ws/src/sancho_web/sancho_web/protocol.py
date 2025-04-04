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

##### TYPES #####
class MessageType(str, Enum):
    RESPONSE = "RESPONSE"
    PROMPT = "PROMPT"
    
MESSAGE_OBJECT = {
    MessageType.RESPONSE: ResponseMessage,
    MessageType.PROMPT: PromptMessage,
}

##### PARSER #####
def parse_message(msg):
    try:
        msg = json.loads(msg)
        return msg.get("type"), msg.get("data")
    except json.JSONDecodeError:
        print("Error: mensaje JSON malformado")
        return None, None
