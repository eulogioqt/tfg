"""TODO: Add module documentation."""
import uuid

from .chunk_protocol import JSONMessage, ChunkMessage


class ChunkManager:
"""TODO: Describe class."""
    def __init__(self, max_chunk_size=256000):
    """TODO: Describe __init__.
Args:
    max_chunk_size (:obj:`Any`): TODO.
"""
        self.max_chunk_size = max_chunk_size

        self.buffers = {}

    def msg_to_chunks(self, full_str) -> list[ChunkMessage]:
    """TODO: Describe msg_to_chunks.
Args:
    full_str (:obj:`Any`): TODO.
"""
        chunks = []
        
        id = str(uuid.uuid4())
        for i in range(0, len(full_str), self.max_chunk_size):
            chunks.append(ChunkMessage(
                id=id,
                chunk_index=i // self.max_chunk_size,
                final=(i + self.max_chunk_size) >= len(full_str),
                data=full_str[i:i + self.max_chunk_size]
            ))

        return chunks

    def chunk_to_msg(self, id, chunk_index, final, data):
    """TODO: Describe chunk_to_msg.
Args:
    id (:obj:`Any`): TODO.
    chunk_index (:obj:`Any`): TODO.
    final (:obj:`Any`): TODO.
    data (:obj:`Any`): TODO.
"""
        if id not in self.buffers:
            self.buffers[id] = []

        self.buffers[id].append((chunk_index, data))

        if final:
            full = "".join([p for _, p in sorted(self.buffers[id], key=lambda x: x[0])])
            del self.buffers[id]
            return full

        return None
