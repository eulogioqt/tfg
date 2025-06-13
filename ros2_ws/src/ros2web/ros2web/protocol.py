"""TODO: Add module documentation."""
import json

from enum import Enum
from abc import ABC, abstractmethod


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
    
class Message(JSONMessage):
"""TODO: Describe class."""
    def __init__(self, data):
    """TODO: Describe __init__.
Args:
    data (:obj:`Any`): TODO.
"""
        self.data = data
    
    def to_dict(self):
    """TODO: Describe to_dict.
"""
        return {
            "type": MessageType.MESSAGE,
            "data": self.data
        }
    
class TopicMessage(JSONMessage):
"""TODO: Describe class."""
    def __init__(self, topic, name, value):
    """TODO: Describe __init__.
Args:
    topic (:obj:`Any`): TODO.
    name (:obj:`Any`): TODO.
    value (:obj:`Any`): TODO.
"""
        self.topic = topic
        self.name = name
        self.value = value

    def to_dict(self):
    """TODO: Describe to_dict.
"""
        return {
            "type": MessageType.TOPIC,
            "data": {
                "topic": self.topic,
                "name": self.name,
                "value": self.value
            }
        }


##### TYPES #####
class MessageType(str, Enum):
"""TODO: Describe class."""
    MESSAGE = "MESSAGE"
    TOPIC = "TOPIC"


MESSAGE_OBJECT = {
    MessageType.MESSAGE: Message,
    MessageType.TOPIC: TopicMessage
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
