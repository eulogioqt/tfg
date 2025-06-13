"""TODO: Add module documentation."""
import json

from enum import Enum
from .protocol import JSONMessage


##### SERVER MESSAGES #####    
class ChunkMessage(JSONMessage):
"""TODO: Describe class."""
    def __init__(self, id, chunk_index, final, data):
    """TODO: Describe __init__.
Args:
    id (:obj:`Any`): TODO.
    chunk_index (:obj:`Any`): TODO.
    final (:obj:`Any`): TODO.
    data (:obj:`Any`): TODO.
"""
        self.id = id,
        self.chunk_index = chunk_index
        self.final = final
        self.data = data
    
    def to_dict(self):
    """TODO: Describe to_dict.
"""
        return {
            "type": ChunkMessageType.CHUNK,
            "id": self.id,
            "chunk_index": self.chunk_index,
            "final": self.final,
            "data": self.data
        }

##### TYPES #####
class ChunkMessageType(str, Enum):
"""TODO: Describe class."""
    CHUNK = "CHUNK"


CHUNK_MESSAGE_OBJECT = {
    ChunkMessageType.CHUNK: ChunkMessage,
}


##### PARSER #####
def parse_chunk_message(msg):
"""TODO: Describe parse_chunk_message.
Args:
    msg (:obj:`Any`): TODO.
"""
    try:
        msg = json.loads(msg)
        return msg.get("type"), msg.get("id"), msg.get("chunk_index"), msg.get("final"), msg.get("data")
    except json.JSONDecodeError:
        print("Error: mensaje JSON malformado")
        return None, None
