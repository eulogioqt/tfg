import websockets
import asyncio
import random
import json

from websockets import WebSocketServerProtocol

async def send_data(websocket: WebSocketServerProtocol, path=None):
    while True:
        n = random.randint(1, 100)
        message_dict = {
            "type": "NUMBER",
            "data": n
        }
        message_json = json.dumps(message_dict)
        
        print(f"Enviando {n} a {websocket.remote_address[0]}:{websocket.remote_address[1]}")
        await websocket.send(message_json)

        await asyncio.sleep(1)

async def main():
    async with websockets.serve(send_data, "localhost", 8765):
        url = f"http://localhost:{8765}"
        print(f"Running on {url}")  # Este mensaje es clave para que VSCode lo detecte

        print(f"Listening on port {8765}...")
        await asyncio.Future()

asyncio.run(main())