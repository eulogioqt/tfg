"""TODO: Add module documentation."""
import json

from enum import Enum
from abc import ABC, abstractmethod


##### CLIENT MESSAGES #####
class PromptMessage():
"""TODO: Describe class."""
    def __init__(self, msg):
    """TODO: Describe __init__.
Args:
    msg (:obj:`Any`): TODO.
"""
        self.id = msg["id"]
        self.chat_id = msg["chatId"]
        self.want_tts = msg["wantTts"]
        self.value = msg["value"]

class AudioPromptMessage():
"""TODO: Describe class."""
    def __init__(self, msg):
    """TODO: Describe __init__.
Args:
    msg (:obj:`Any`): TODO.
"""
        self.id = msg["id"]
        self.chat_id = msg["chatId"]
        self.want_tts = msg["wantTts"]
        self.audio = msg["audio"]
        self.sample_rate = msg["sampleRate"]

class TranscriptionRequestMessage():
"""TODO: Describe class."""
    def __init__(self, msg):
    """TODO: Describe __init__.
Args:
    msg (:obj:`Any`): TODO.
"""
        self.id = msg["id"]
        self.audio = msg["audio"]
        self.sample_rate = msg["sampleRate"]

##### SERVER MESSAGES #####
class JSONMessage(ABC):
    @abstractmethod
"""TODO: Describe class."""
    def to_dict(self) -> dict:
    """TODO: Describe to_dict.
"""
        pass

    def to_json(self) -> str:
    """TODO: Describe to_json.
"""
        return json.dumps(self.to_dict())


class ResponseMessage(JSONMessage):
"""TODO: Describe class."""
    def __init__(self, id, value, method, intent, arguments, provider, model):
    """TODO: Describe __init__.
Args:
    id (:obj:`Any`): TODO.
    value (:obj:`Any`): TODO.
    method (:obj:`Any`): TODO.
    intent (:obj:`Any`): TODO.
    arguments (:obj:`Any`): TODO.
    provider (:obj:`Any`): TODO.
    model (:obj:`Any`): TODO.
"""
        self.id = id
        self.value = value
        self.method = method
        self.intent = intent
        self.arguments = arguments
        self.provider = provider
        self.model = model
    
    def to_dict(self):
    """TODO: Describe to_dict.
"""
        return {
            "type": MessageType.RESPONSE,
            "data": {
                "id": self.id,
                "method": self.method,
                "intent": self.intent,
                "arguments": self.arguments,
                "provider": self.provider,
                "model": self.model,
                "value": self.value
            }
        }


class FaceprintEventMessage(JSONMessage):
"""TODO: Describe class."""
    class Event(str, Enum):
    """TODO: Describe class."""
        CREATE = "CREATE"
        UPDATE = "UPDATE"
        DELETE = "DELETE"

    def __init__(self, event, id):
    """TODO: Describe __init__.
Args:
    event (:obj:`Any`): TODO.
    id (:obj:`Any`): TODO.
"""
        self.event = event
        self.id = id
    
    def to_dict(self):
    """TODO: Describe to_dict.
"""
        return {
            "type": MessageType.FACEPRINT_EVENT,
            "data": {
                "event": self.event,
                "id": self.id
            }
        }

class PromptTranscriptionMessage(JSONMessage):
"""TODO: Describe class."""
    def __init__(self, id, value, model):
    """TODO: Describe __init__.
Args:
    id (:obj:`Any`): TODO.
    value (:obj:`Any`): TODO.
    model (:obj:`Any`): TODO.
"""
        self.id = id
        self.value = value
        self.model = model
    
    def to_dict(self):
    """TODO: Describe to_dict.
"""
        return {
            "type": MessageType.PROMPT_TRANSCRIPTION,
            "data": {
                "id": self.id,
                "model": self.model,
                "value": self.value
            }
        }

class AudioResponseMessage(JSONMessage):
"""TODO: Describe class."""
    def __init__(self, id, audio, sample_rate, model, speaker):
    """TODO: Describe __init__.
Args:
    id (:obj:`Any`): TODO.
    audio (:obj:`Any`): TODO.
    sample_rate (:obj:`Any`): TODO.
    model (:obj:`Any`): TODO.
    speaker (:obj:`Any`): TODO.
"""
        self.id = id
        self.audio = list(audio)
        self.sample_rate = sample_rate
        self.model = model
        self.speaker = speaker
        
    def to_dict(self):
    """TODO: Describe to_dict.
"""
        return {
            "type": MessageType.AUDIO_RESPONSE,
            "data": {
                "id": self.id,
                "model": self.model,
                "speaker": self.speaker,
                "audio": self.audio,
                "sampleRate": self.sample_rate
            }
        }

##### TYPES #####
class MessageType(str, Enum):
"""TODO: Describe class."""
    PROMPT = "PROMPT" # Client -> Server
    AUDIO_PROMPT = "AUDIO_PROMPT" # Client -> Server
    TRANSCRIPTION_REQUEST = "TRANSCRIPTION_REQUEST" # Client -> Server

    RESPONSE = "RESPONSE" # Server -> Client
    FACEPRINT_EVENT = "FACEPRINT_EVENT" # Server -> Client
    PROMPT_TRANSCRIPTION = "PROMPT_TRANSCRIPTION" # Server -> Client
    AUDIO_RESPONSE = "AUDIO_RESPONSE" # Server -> Client
    
MESSAGE_OBJECT = {
    MessageType.PROMPT: PromptMessage,
    MessageType.AUDIO_PROMPT: AudioPromptMessage,
    MessageType.TRANSCRIPTION_REQUEST: TranscriptionRequestMessage,

    MessageType.RESPONSE: ResponseMessage,
    MessageType.FACEPRINT_EVENT: FaceprintEventMessage,
    MessageType.PROMPT_TRANSCRIPTION: PromptTranscriptionMessage,
    MessageType.AUDIO_RESPONSE: AudioResponseMessage
}


##### PARSER #####
def parse_message(msg):
"""TODO: Describe parse_message.
Args:
    msg (:obj:`Any`): TODO.
"""
    try:
        msg = json.loads(msg)
        return msg.get("type"), msg.get("data")
    except json.JSONDecodeError:
        print("Error: mensaje JSON malformado")
        return None, None
