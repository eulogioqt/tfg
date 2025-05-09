import json

from enum import Enum
from .protocol import JSONMessage


##### SERVER MESSAGES #####    
class ChunkMessage(JSONMessage):
    def __init__(self, id, chunk_index, final, data):
        self.id = id,
        self.chunk_index = chunk_index
        self.final = final
        self.data = data
    
    def to_json(self):
        message = {
            "type": ChunkMessageType.CHUNK,
            "id": self.id,
            "chunk_index": self.chunk_index,
            "final": self.final,
            "data": self.data
        }

        return json.dumps(message)

##### TYPES #####
class ChunkMessageType(str, Enum):
    CHUNK = "CHUNK"


CHUNK_MESSAGE_OBJECT = {
    ChunkMessageType.CHUNK: ChunkMessage,
}


##### PARSER #####
def parse_chunk_message(msg):
    try:
        msg = json.loads(msg)
        return msg.get("type"), msg.get("id"), msg.get("chunk_id"), msg.get("final"), msg.get("data")
    except json.JSONDecodeError:
        print("Error: mensaje JSON malformado")
        return None, None
