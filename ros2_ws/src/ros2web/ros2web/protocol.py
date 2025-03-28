import json

from enum import Enum


##### SERVER MESSAGES #####
class Message():
    def __init__(self, data):
        self.data = data
    
    def to_json(self):
        message = {
            "type": MessageType.MESSAGE,
            "data": self.data
        }

        return json.dumps(message)
    
#class TopicMessage()


##### TYPES #####
class MessageType(str, Enum):
    MESSAGE = "MESSAGE"
    TOPIC = "TOPIC"


MESSAGE_OBJECT = {
    MessageType.MESSAGE: Message,
}


##### PARSER #####
def parse_message(msg):
    try:
        msg = json.loads(msg)
        return msg.get("type"), msg.get("data")
    except json.JSONDecodeError:
        print("Error: mensaje JSON malformado")
        return None, None
