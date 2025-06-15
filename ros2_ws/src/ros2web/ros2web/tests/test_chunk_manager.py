from ros2web.chunk_manager import ChunkManager

# Black-box: verify messages are fragmented correctly

def test_msg_to_chunks_basic():
    manager = ChunkManager(max_chunk_size=4)
    chunks = manager.msg_to_chunks("abcdef")
    assert len(chunks) == 2
    assert chunks[0].chunk_index == 0
    assert chunks[0].final is False
    assert chunks[1].chunk_index == 1
    assert chunks[1].final is True


# White-box: ensure chunks are reassembled and buffer cleared

def test_chunk_to_msg_reassembles():
    manager = ChunkManager(max_chunk_size=3)
    msg_id = "1"
    part = manager.chunk_to_msg(msg_id, 0, False, "abc")
    assert part is None
    final = manager.chunk_to_msg(msg_id, 1, True, "def")
    assert final == "abcdef"
    assert msg_id not in manager.buffers
