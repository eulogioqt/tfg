import json

from enum import Enum
from abc import ABC, abstractmethod


##### CLIENT MESSAGES #####
class PromptMessage():
    def __init__(self, msg):
        self.id = msg["id"]
        self.value = msg["value"]

class AudioPromptMessage():
    def __init__(self, msg):
        self.id = msg["id"]
        self.audio = msg["audio"]
        self.sample_rate = msg["sample_rate"]

##### SERVER MESSAGES #####
class JSONMessage(ABC):
    @abstractmethod
    def to_dict(self) -> dict:
        pass

    def to_json(self) -> str:
        return json.dumps(self.to_dict())


class ResponseMessage(JSONMessage):
    def __init__(self, id, response):
        self.id = id
        self.response = response
    
    def to_dict(self):
        return {
            "type": MessageType.RESPONSE,
            "data": {
                "id": self.id,
                "value": self.response
            }
        }


class FaceprintEventMessage(JSONMessage):
    class Event(str, Enum):
        CREATE = "CREATE"
        UPDATE = "UPDATE"
        DELETE = "DELETE"

    def __init__(self, event, id):
        self.event = event
        self.id = id
    
    def to_dict(self):
        return {
            "type": MessageType.FACEPRINT_EVENT,
            "data": {
                "event": self.event,
                "id": self.id
            }
        }

class PromptTranscriptionMessage(JSONMessage):
    def __init__(self, id, value):
        self.id = id
        self.value = value
    
    def to_dict(self):
        return {
            "type": MessageType.PROMPT_TRANSCRIPTION,
            "data": {
                "id": self.id,
                "value": self.value
            }
        }

class AudioResponseMessage(JSONMessage):
    def __init__(self, id, audio, sample_rate):
        self.id = id
        self.audio = audio
        self.sample_rate = sample_rate
    
    def to_dict(self):
        return {
            "type": MessageType.AUDIO_RESPONSE,
            "data": {
                "id": self.id,
                "audio": self.audio,
                "sample_rate": self.sample_rate
            }
        }

##### TYPES #####
class MessageType(str, Enum):
    PROMPT = "PROMPT" # Client -> Server
    AUDIO_PROMPT = "AUDIO_PROMPT" # Client -> Server

    RESPONSE = "RESPONSE" # Server -> Client
    FACEPRINT_EVENT = "FACEPRINT_EVENT" # Server -> Client
    PROMPT_TRANSCRIPTION = "PROMPT_TRANSCRIPTION" # Server -> Client
    AUDIO_RESPONSE = "AUDIO_RESPONSE" # Server -> Client
    
MESSAGE_OBJECT = {
    MessageType.PROMPT: PromptMessage,
    MessageType.AUDIO_PROMPT: AudioPromptMessage,

    MessageType.RESPONSE: ResponseMessage,
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
