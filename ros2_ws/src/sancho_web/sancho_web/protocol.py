import json

from enum import Enum
from abc import ABC, abstractmethod


##### CLIENT MESSAGES #####
class PromptMessage():
    def __init__(self, msg):
        self.id = msg["id"]
        self.value = msg["value"]

class AudioPromptChunkMessage():
    def __init__(self, msg):
        self.id = msg["id"]
        self.chunk_index = msg["chunk_index"]
        self.final = msg["final"]
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

class AudioResponseChunkMessage(JSONMessage):
    def __init__(self, id, chunk_index, final, audio, sample_rate):
        self.id = id
        self.chunk_index = chunk_index
        self.final = final
        self.audio = audio
        self.sample_rate = sample_rate
        
    def to_dict(self):
        return {
            "type": MessageType.AUDIO_RESPONSE_CHUNK,
            "data": {
                "id": self.id,
                "chunk_index": self.chunk_index,
                "final": self.final,
                "audio": self.audio,
                "sample_rate": self.sample_rate
            }
        }

##### TYPES #####
class MessageType(str, Enum):
    PROMPT = "PROMPT" # Client -> Server
    AUDIO_PROMPT_CHUNK = "AUDIO_PROMPT_CHUNK" # Client -> Server

    RESPONSE = "RESPONSE" # Server -> Client
    FACEPRINT_EVENT = "FACEPRINT_EVENT" # Server -> Client
    PROMPT_TRANSCRIPTION = "PROMPT_TRANSCRIPTION" # Server -> Client
    AUDIO_RESPONSE_CHUNK = "AUDIO_RESPONSE_CHUNK" # Server -> Client
    
MESSAGE_OBJECT = {
    MessageType.PROMPT: PromptMessage,
    MessageType.AUDIO_PROMPT_CHUNK: AudioPromptChunkMessage,

    MessageType.RESPONSE: ResponseMessage,
    MessageType.FACEPRINT_EVENT: FaceprintEventMessage,
    MessageType.PROMPT_TRANSCRIPTION: PromptTranscriptionMessage,
    MessageType.AUDIO_RESPONSE_CHUNK: AudioResponseChunkMessage
}


##### PARSER #####
def parse_message(msg):
    try:
        msg = json.loads(msg)
        return msg.get("type"), msg.get("data")
    except json.JSONDecodeError:
        print("Error: mensaje JSON malformado")
        return None, None
