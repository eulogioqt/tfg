import uuid

from .chunk_protocol import JSONMessage, ChunkMessage, parse_chunk_message


class ChunkManager:
    def __init__(self, max_chunk_size=64000):
        self.max_chunk_size = max_chunk_size

        self.buffers = {}

    def msg_to_chunks(self, full_str) -> list[JSONMessage]:
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

    def chunk_to_msg(self, chunk):
        _, chunk_id, index, final, payload = parse_chunk_message(chunk)

        if chunk_id not in self.buffers:
            self.buffers[chunk_id] = []

        self.buffers[chunk_id].append((index, payload))

        if final:
            full = "".join([p for _, p in sorted(self.buffers[chunk_id], key=lambda x: x[0])])
            del self.buffers[chunk_id]
            return full

        return None
