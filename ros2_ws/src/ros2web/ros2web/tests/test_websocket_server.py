import asyncio
import pytest
from unittest.mock import AsyncMock

from ros2web.websocket_server import WebSocketServer


class DummyWebSocket:
    def __init__(self, messages):
        self._messages = list(messages)
        self.sent = []
        self.remote_address = ("127.0.0.1", 1234)

    async def send(self, msg):
        self.sent.append(msg)

    def __aiter__(self):
        return self

    async def __anext__(self):
        if self._messages:
            return self._messages.pop(0)
        raise StopAsyncIteration


# Hybrid: handler should register client and process messages
@pytest.mark.asyncio
async def test_handler_processes_messages():
    events = []
    server = WebSocketServer(
        on_message=lambda key, msg: events.append(("msg", key, msg)),
        on_user_connect=lambda key: events.append(("connect", key)),
        on_user_disconnect=lambda key: events.append(("disconnect", key)),
    )

    ws = DummyWebSocket(["hello"])
    await server._handler(ws)

    assert events == [
        ("connect", "127.0.0.1:1234"),
        ("msg", "127.0.0.1:1234", "hello"),
        ("disconnect", "127.0.0.1:1234"),
    ]
    assert server.get_connection_count() == 0


# White-box: _send_message sends via underlying websocket
@pytest.mark.asyncio
async def test_send_message():
    server = WebSocketServer(None, None, None)
    client = AsyncMock()
    await server._send_message(client, "data")
    client.send.assert_called_once_with("data")
